// ecu regulateur de vitesse - tp temps reel
// pid + failsafe sur le core 1, rx uart sur le core 0

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "protocol.h"

// bus sur uart2 pour pas melanger les trames avec les logs uart0
#define BUS_UART     UART_NUM_2
#define BUS_TX_PIN   17
#define BUS_RX_PIN   16
#define RX_BUF       1024
#define TX_BUF       1024

#define ERR_GPIO     4
#define LED_GPIO     2

#define KP   1.0f
#define KI   0.1f
#define KD   0.01f
#define DT   0.1f

#define CAUSE_TIMEOUT  1
#define CAUSE_GPIO     2

static const char *TAG = "ecu";

static QueueHandle_t   g_cmd_queue;
static SemaphoreHandle_t g_tx_mutex;
static TimerHandle_t   g_wdog;
static TaskHandle_t    g_failsafe_task;

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile int  g_mode = MODE_OFF;
static volatile bool g_failsafe = false;

typedef struct {
    uint32_t rx_valid;
    uint32_t rx_crc_err;
    uint32_t rx_dropped;
    uint32_t tx_output;
    uint32_t rx_unknown;
} stats_t;
static stats_t g_stats;

typedef struct {
    uint8_t type;
    float   value;
} cmd_t;

static inline void stat_inc(uint32_t *field)
{
    taskENTER_CRITICAL(&g_mux);
    (*field)++;
    taskEXIT_CRITICAL(&g_mux);
}

// mutex = pas d'entrelacement de trames (nf03) + heritage de prio (nf04)
static void tx_frame(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[5 + MAX_PAYLOAD];
    if (len > MAX_PAYLOAD) {
        len = MAX_PAYLOAD;
    }
    size_t n = build_frame(buf, type, payload, len);

    xSemaphoreTake(g_tx_mutex, portMAX_DELAY);
    uart_write_bytes(BUS_UART, (const char *)buf, n);
    xSemaphoreGive(g_tx_mutex);
}

static void send_output(float v)
{
    tx_frame(MSG_OUTPUT, (const uint8_t *)&v, sizeof(float));
    stat_inc(&g_stats.tx_output);
}

static void send_alarm(const char *txt)
{
    tx_frame(MSG_ALARM, (const uint8_t *)txt, (uint16_t)strlen(txt));
}

enum { P_START, P_LEN, P_DATA, P_CRC };

typedef struct {
    int      state;
    uint16_t len;
    uint16_t idx;
    uint8_t  lenb[2];
    uint8_t  data[1 + MAX_PAYLOAD];
} parser_t;

static void push_cmd(uint8_t type, const uint8_t *payload)
{
    cmd_t c;
    c.type = type;
    memcpy(&c.value, payload, sizeof(float));
    if (xQueueSend(g_cmd_queue, &c, 0) != pdTRUE) {
        stat_inc(&g_stats.rx_dropped);
    }
}

static void set_mode(uint8_t m)
{
    if (m > MODE_AUTO) {
        return;
    }
    taskENTER_CRITICAL(&g_mux);
    g_mode = m;
    g_failsafe = false;   // relance apres un failsafe (exigence 06)
    taskEXIT_CRITICAL(&g_mux);

    gpio_set_level(LED_GPIO, 0);
}

static void handle_frame(uint8_t type, const uint8_t *payload, uint16_t plen)
{
    bool ok = false;

    switch (type) {
    case MSG_SETPOINT:
        if (plen == 4) { push_cmd(MSG_SETPOINT, payload); ok = true; }
        break;
    case MSG_SPEED:
        if (plen == 4) { push_cmd(MSG_SPEED, payload); ok = true; }
        break;
    case MSG_MODE_SET:
        if (plen == 1) { set_mode(payload[0]); ok = true; }
        break;
    default:
        stat_inc(&g_stats.rx_unknown);
        return;
    }

    if (ok) {
        stat_inc(&g_stats.rx_valid);
        xTimerReset(g_wdog, 0);
    } else {
        stat_inc(&g_stats.rx_crc_err);
    }
}

static void parser_feed(parser_t *p, uint8_t b)
{
    switch (p->state) {
    case P_START:
        if (b == FRAME_START) {
            p->state = P_LEN;
            p->idx = 0;
        }
        break;

    case P_LEN:
        p->lenb[p->idx++] = b;
        if (p->idx == 2) {
            p->len = p->lenb[0] | (p->lenb[1] << 8);
            if (p->len < 1 || p->len > 1 + MAX_PAYLOAD) {
                stat_inc(&g_stats.rx_crc_err);
                p->state = P_START;
            } else {
                p->idx = 0;
                p->state = P_DATA;
            }
        }
        break;

    case P_DATA:
        p->data[p->idx++] = b;
        if (p->idx == p->len) {
            p->state = P_CRC;
        }
        break;

    case P_CRC:
        {
            uint8_t want = crc_xor(p->lenb, 2) ^ crc_xor(p->data, p->len);
            if (want == b) {
                handle_frame(p->data[0], &p->data[1], p->len - 1);
            } else {
                stat_inc(&g_stats.rx_crc_err);
            }
            p->state = P_START;
        }
        break;
    }
}

static void task_rx(void *arg)
{
    parser_t p = { .state = P_START };
    uint8_t buf[128];

    for (;;) {
        int n = uart_read_bytes(BUS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        //ESP_LOGI(TAG, "rx %d octets", n);
        for (int i = 0; i < n; i++) {
            parser_feed(&p, buf[i]);
        }
    }
}

static void task_control(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    float setpoint = 0.0f, speed = 0.0f;
    float integral = 0.0f, prev_err = 0.0f;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));

        cmd_t c;
        while (xQueueReceive(g_cmd_queue, &c, 0) == pdTRUE) {
            if (c.type == MSG_SETPOINT) {
                setpoint = c.value;
            } else if (c.type == MSG_SPEED) {
                speed = c.value;
            }
        }

        taskENTER_CRITICAL(&g_mux);
        int mode = g_mode;
        bool fs = g_failsafe;
        taskEXIT_CRITICAL(&g_mux);

        if (fs || mode != MODE_AUTO) {
            integral = 0.0f;
            prev_err = setpoint - speed;
            continue;
        }

        // pid discret + anti windup (cf exemple sujet)
        float err = setpoint - speed;
        float integ_cand = integral + err * DT;
        float der = (err - prev_err) / DT;
        float out = (KP * err) + (KI * integ_cand) + (KD * der);

        if (out > 255.0f) {
            out = 255.0f;
            if (err > 0.0f) integ_cand = integral;
        } else if (out < 0.0f) {
            out = 0.0f;
            if (err < 0.0f) integ_cand = integral;
        }

        integral = integ_cand;
        prev_err = err;

        send_output(out);
    }
}

static void task_stats(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));

        uint32_t snap[5];
        taskENTER_CRITICAL(&g_mux);
        snap[0] = g_stats.rx_valid;
        snap[1] = g_stats.rx_crc_err;
        snap[2] = g_stats.rx_dropped;
        snap[3] = g_stats.tx_output;
        snap[4] = g_stats.rx_unknown;
        taskEXIT_CRITICAL(&g_mux);

        tx_frame(MSG_STATS, (const uint8_t *)snap, sizeof(snap));

        // %u et pas %lu sinon -Werror=format rale (uint32_t = long unsigned sur xtensa)
        ESP_LOGI(TAG, "mode=%d fs=%d | rxok=%u crc=%u drop=%u out=%u unk=%u | heap=%u",
                 g_mode, (int)g_failsafe,
                 (unsigned)snap[0], (unsigned)snap[1], (unsigned)snap[2],
                 (unsigned)snap[3], (unsigned)snap[4],
                 (unsigned)esp_get_free_heap_size());
    }
}

// prio max : preempte tout, donc coupe en moins de 5 ms sans attendre le pid
static void task_failsafe(void *arg)
{
    uint32_t cause;

    for (;;) {
        xTaskNotifyWait(0, 0xffffffff, &cause, portMAX_DELAY);

        taskENTER_CRITICAL(&g_mux);
        g_mode = MODE_OFF;
        g_failsafe = true;
        taskEXIT_CRITICAL(&g_mux);

        gpio_set_level(LED_GPIO, 1);
        send_output(0.0f);

        if (cause == CAUSE_GPIO) {
            send_alarm("gpio erreur");
        } else {
            send_alarm("timeout rx");
        }

        ESP_LOGW(TAG, "failsafe cause=%u", (unsigned)cause);
    }
}

// juste une notification, le reste est fait dans la tache
static void IRAM_ATTR err_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(g_failsafe_task, CAUSE_GPIO, eSetValueWithOverwrite, &hp);
    portYIELD_FROM_ISR(hp);
}

static void wdog_cb(TimerHandle_t t)
{
    xTaskNotify(g_failsafe_task, CAUSE_TIMEOUT, eSetValueWithOverwrite);
}

static void bus_uart_init(void)
{
    uart_config_t uc = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(BUS_UART, &uc);
    uart_set_pin(BUS_UART, BUS_TX_PIN, BUS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(BUS_UART, RX_BUF, TX_BUF, 0, NULL, 0);
}

static void gpio_init_all(void)
{
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 0);

    gpio_config_t err = {
        .pin_bit_mask = 1ULL << ERR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&err);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ERR_GPIO, err_isr, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "demarrage ecu");

    bus_uart_init();

    g_cmd_queue = xQueueCreate(16, sizeof(cmd_t));
    g_tx_mutex  = xSemaphoreCreateMutex();
    g_wdog      = xTimerCreate("wdog", pdMS_TO_TICKS(2000), pdFALSE, NULL, wdog_cb);

    g_mode = MODE_OFF;
    g_failsafe = false;

    xTaskCreatePinnedToCore(task_failsafe, "failsafe", 2048, NULL, 4, &g_failsafe_task, 1);
    xTaskCreatePinnedToCore(task_control,  "control",  2560, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(task_stats,    "stats",    3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_rx,       "rx",       3072, NULL, 2, NULL, 0);

    gpio_init_all();
    xTimerStart(g_wdog, 0);
}
