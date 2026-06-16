#include "protocol.h"
#include <string.h>

uint8_t crc_xor(const uint8_t *data, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}

size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t payload_len)
{
    // len = taille de (type + payload), cf sujet
    uint16_t len = payload_len + 1;

    out[0] = FRAME_START;
    out[1] = len & 0xff;          // little endian
    out[2] = (len >> 8) & 0xff;
    out[3] = type;

    if (payload_len > 0 && payload != NULL) {
        memcpy(&out[4], payload, payload_len);
    }

    // le crc couvre tout sauf le start, donc a partir de out[1]
    // soit len(2) + type(1) + payload = 3 + payload_len octets
    out[4 + payload_len] = crc_xor(&out[1], 3 + payload_len);

    return 5 + payload_len;
}
