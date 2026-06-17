#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define FRAME_START  0xAA
#define MAX_PAYLOAD  64

#define MSG_SETPOINT 0x01
#define MSG_SPEED    0x02
#define MSG_MODE_SET 0x05
#define MSG_OUTPUT   0x80
#define MSG_STATS    0x83
#define MSG_ALARM    0x85
#define MSG_DBG      0xff

#define MODE_OFF     0
#define MODE_MANUAL  1
#define MODE_AUTO    2

uint8_t crc_xor(const uint8_t *data, size_t len);
size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t payload_len);

#endif
