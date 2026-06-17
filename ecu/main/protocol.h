#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// octet de debut de trame
#define FRAME_START  0xAA

// taille max du payload qu'on accepte (garde fou contre les trames absurdes)
#define MAX_PAYLOAD  64

// ids des messages (cf tableau du sujet)
#define MSG_SETPOINT 0x01
#define MSG_SPEED    0x02
#define MSG_MODE_SET 0x05
#define MSG_OUTPUT   0x80
#define MSG_STATS    0x83
#define MSG_ALARM    0x85
#define MSG_DBG      0xff

// modes de fonctionnement
#define MODE_OFF     0
#define MODE_MANUAL  1
#define MODE_AUTO    2

// crc = xor de tous les octets de la trame sauf le start
uint8_t crc_xor(const uint8_t *data, size_t len);

// construit la trame dans out, retourne sa taille (out doit faire 5 + payload_len mini)
size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t payload_len);

#endif
