// ecu regulateur de vitesse - tp temps reel epita
// archi multi-taches sous freertos (cf rapport pour le tableau des taches)
//
// idee : une tache = une responsabilite (cf cm6), reliees par des primitives
// freertos (queue, mutex, notification) et jamais par des globales nues.
// les taches critiques (pid + failsafe) sont sur le core 1, la reception
// uart sur le core 0 -> isolation du flood vis a vis de la regulation.

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
#include "esp_system.h"   // esp_get_free_heap_size
#include "esp_attr.h"     // IRAM_ATTR

#include "protocol.h"

// ----- config materielle -----
// le bus ecu est sur uart2 avec des pins dedies, pour ne pas melanger
// les trames binaires avec les logs console (qui restent sur uart0)
#define BUS_UART     UART_NUM_2
#define BUS_TX_PIN   17
#define BUS_RX_PIN   16
#define RX_BUF       1024
#define TX_BUF       1024

#define ERR_GPIO     4    // entree erreur externe (front -> failsafe)
#define LED_GPIO     2    // led failsafe (led onboard de la plupart des devkit)

// ----- coeffs du pid (cf sujet, ajustables) -----
#define KP   1.0f
#define KI   0.1f
#define KD   0.01f
#define DT   0.1f         // 100 ms

// causes de failsafe passees dans la notification
#define CAUSE_TIMEOUT  1
#define CAUSE_GPIO     2

static const char *TAG = "ecu";

// ----- objets freertos partages -----
static QueueHandle_t   g_cmd_queue;     // commandes decodees parser -> pid
static SemaphoreHandle_t g_tx_mutex;    // protege l'emission uart (1 seul ecrivain a la fois)
static TimerHandle_t   g_wdog;          // watchdog rx (2s sans trame valide)
static TaskHandle_t    g_failsafe_task; // pour notifier depuis isr / timer

// spinlock pour les sections critiques (etat de supervision + compteurs)
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ----- etat de supervision (lu par le pid, ecrit par rx et failsafe) -----
static volatile int  g_mode = MODE_OFF;
static volatile bool g_failsafe = false;

// ----- compteurs telemetrie -----
typedef struct {
    uint32_t rx_valid;
    uint32_t rx_crc_err;
    uint32_t rx_dropped;
    uint32_t tx_output;
    uint32_t rx_unknown;
} stats_t;
static stats_t g_stats;

// commande decodee envoyee au pid
typedef struct {
    uint8_t type;   // MSG_SETPOINT ou MSG_SPEED
    float   value;
} cmd_t;

// =====================================================================
// helpers compteurs (acces tres courts -> section critique, cf cm4)
// =====================================================================
static inline void stat_inc(uint32_t *field)
{
    taskENTER_CRITICAL(&g_mux);
    (*field)++;
    taskEXIT_CRITICAL(&g_mux);
}

// =====================================================================
// emission uart
// le mutex garantit qu'on n'entrelace pas deux trames (integrite, nf03)
// et son heritage de priorite evite l'inversion de prio (nf04).
// la section protegee est tres courte : uart_write_bytes ne fait que
// copier dans le ring buffer tx du driver, il n'attend pas la fin d'envoi.
// =====================================================================
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

// =====================================================================
// parser de trames (machine a etats, octet par octet)
// gere la fragmentation (on garde l'etat entre deux lectures) et les
// trames collees (on repart en P_START apres chaque trame complete).
// =====================================================================
enum { P_START, P_LEN, P_DATA, P_CRC };

typedef struct {
    int      state;
    uint16_t len;
    uint16_t idx;
    uint8_t  lenb[2];
    uint8_t  data[1 + MAX_PAYLOAD];   // type + payload
} parser_t;

// transmet une commande au pid (drop si la queue est pleine -> compteur)
static void push_cmd(uint8_t type, const uint8_t *payload)
{
    cmd_t c;
    c.type = type;
    memcpy(&c.value, payload, sizeof(float));
    if (xQueueSend(g_cmd_queue, &c, 0) != pdTRUE) {
        stat_inc(&g_stats.rx_dropped);
    }
}

// applique un changement de mode + gere la reprise apres failsafe
static void set_mode(uint8_t m)
{
    if (m > MODE_AUTO) {
        return;   // valeur de mode invalide
    }
    taskENTER_CRITICAL(&g_mux);
    g_mode = m;
    g_failsafe = false;   // un nouveau mode_set relance le systeme (exigence 06)
    taskEXIT_CRITICAL(&g_mux);

    gpio_set_level(LED_GPIO, 0);   // on sort de l'etat d'alarme
}

// traite une trame complete et valide (crc deja verifie)
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
        // id inconnu -> on ignore silencieusement (cf sujet)
        stat_inc(&g_stats.rx_unknown);
        return;
    }

    if (ok) {
        stat_inc(&g_stats.rx_valid);
        // on a recu une vraie trame -> on rearme le watchdog rx
        xTimerReset(g_wdog, 0);
    } else {
        // type connu mais payload de mauvaise taille = trame malformee
        stat_inc(&g_stats.rx_crc_err);
    }
}

// alimente la machine a etats avec un octet
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
            // len incoherente -> on jette et on resynchronise
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
            // crc attendu = xor(len_bytes) ^ xor(type+payload)
            uint8_t want = crc_xor(p->lenb, 2) ^ crc_xor(p->data, p->len);
            if (want == b) {
                handle_frame(p->data[0], &p->data[1], p->len - 1);
            } else {
                stat_inc(&g_stats.rx_crc_err);   // crc invalide -> rejet silencieux
            }
            p->state = P_START;
        }
        break;
    }
}

// =====================================================================
// tache rx (parser) - prio moyenne, core 0
// lit le bus uart par blocs et alimente la machine a etats
// =====================================================================
static void task_rx(void *arg)
{
    parser_t p = { .state = P_START };
    uint8_t buf[128];

    for (;;) {
        int n = uart_read_bytes(BUS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            parser_feed(&p, buf[i]);
        }
    }
}

// =====================================================================
// tache control (pid) - prio haute, core 1
// periode stricte de 100 ms via vTaskDelayUntil (pas de derive, cf cm2)
// =====================================================================
static void task_control(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    float setpoint = 0.0f, speed = 0.0f;
    float integral = 0.0f, prev_err = 0.0f;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));

        // on vide la queue pour recuperer les dernieres consignes / vitesses
        cmd_t c;
        while (xQueueReceive(g_cmd_queue, &c, 0) == pdTRUE) {
            if (c.type == MSG_SETPOINT) {
                setpoint = c.value;
            } else if (c.type == MSG_SPEED) {
                speed = c.value;
            }
        }

        // lecture de l'etat de supervision
        taskENTER_CRITICAL(&g_mux);
        int mode = g_mode;
        bool fs = g_failsafe;
        taskEXIT_CRITICAL(&g_mux);

        if (fs || mode != MODE_AUTO) {
            // hors auto : pas de regulation, on remet l'integrale a zero
            // pour repartir proprement a la prochaine reprise
            integral = 0.0f;
            prev_err = setpoint - speed;
            continue;   // on emet output uniquement en mode auto
        }

        // pid discret
        float err = setpoint - speed;
        float integ_cand = integral + err * DT;
        float der = (err - prev_err) / DT;
        float out = (KP * err) + (KI * integ_cand) + (KD * der);

        // saturation [0,255] + anti windup (gel de l'integrale, cf exemple sujet)
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

// =====================================================================
// tache stats (telemetrie) - prio basse, core 1
// emet les compteurs toutes les secondes + un log de monitoring console
// =====================================================================
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

        // trame stats sur le bus (uint32 * n, little endian)
        tx_frame(MSG_STATS, (const uint8_t *)snap, sizeof(snap));

        // monitoring console (uart0) : compteurs + heap restant
        // cast en unsigned pour le format : uint32_t = long unsigned sur xtensa
        ESP_LOGI(TAG, "mode=%d fs=%d | rxok=%u crc=%u drop=%u out=%u unk=%u | heap=%u",
                 g_mode, (int)g_failsafe,
                 (unsigned)snap[0], (unsigned)snap[1], (unsigned)snap[2],
                 (unsigned)snap[3], (unsigned)snap[4],
                 (unsigned)esp_get_free_heap_size());
    }
}

// =====================================================================
// tache failsafe - prio max, core 1
// reveillee par notification (front gpio ou timeout watchdog). comme elle
// est la plus prioritaire elle preempte tout -> temps de reponse << 5 ms
// =====================================================================
static void task_failsafe(void *arg)
{
    uint32_t cause;

    for (;;) {
        xTaskNotifyWait(0, 0xffffffff, &cause, portMAX_DELAY);

        // 1. on coupe : mode off + flag failsafe
        taskENTER_CRITICAL(&g_mux);
        g_mode = MODE_OFF;
        g_failsafe = true;
        taskEXIT_CRITICAL(&g_mux);

        // 2. led d'alarme
        gpio_set_level(LED_GPIO, 1);

        // 3. on force la commande moteur a 0 tout de suite, sans attendre
        //    le prochain cycle du pid
        send_output(0.0f);

        // 4. alarme avec la cause
        if (cause == CAUSE_GPIO) {
            send_alarm("gpio erreur");
        } else {
            send_alarm("timeout rx");
        }

        ESP_LOGW(TAG, "failsafe cause=%u", (unsigned)cause);
    }
}

// =====================================================================
// isr / callbacks
// =====================================================================

// isr du gpio d'erreur : courte, juste une notification (deferred work cm2)
static void IRAM_ATTR err_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(g_failsafe_task, CAUSE_GPIO, eSetValueWithOverwrite, &hp);
    portYIELD_FROM_ISR(hp);
}

// callback du watchdog rx : aucune trame valide depuis 2s -> failsafe
// (s'execute dans la timer daemon task, donc court et non bloquant)
static void wdog_cb(TimerHandle_t t)
{
    xTaskNotify(g_failsafe_task, CAUSE_TIMEOUT, eSetValueWithOverwrite);
}

// =====================================================================
// init des peripheriques
// =====================================================================
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
    // led de sortie
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 0);

    // entree erreur externe, interruption sur front descendant
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

// =====================================================================
// point d'entree
// =====================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "demarrage ecu");

    bus_uart_init();

    // objets freertos (avant la creation des taches qui les utilisent)
    g_cmd_queue = xQueueCreate(16, sizeof(cmd_t));
    g_tx_mutex  = xSemaphoreCreateMutex();
    g_wdog      = xTimerCreate("wdog", pdMS_TO_TICKS(2000), pdFALSE, NULL, wdog_cb);

    g_mode = MODE_OFF;
    g_failsafe = false;

    // creation des taches (cf tableau des taches du rapport)
    // critiques (failsafe, pid, stats) sur le core 1, reception sur le core 0
    xTaskCreatePinnedToCore(task_failsafe, "failsafe", 2048, NULL, 4, &g_failsafe_task, 1);
    xTaskCreatePinnedToCore(task_control,  "control",  2560, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(task_stats,    "stats",    3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_rx,       "rx",       3072, NULL, 2, NULL, 0);

    // une fois la tache failsafe creee on peut brancher l'isr gpio
    gpio_init_all();

    // on arme le watchdog : si aucune trame n'arrive jamais, failsafe a 2s
    xTimerStart(g_wdog, 0);
}
