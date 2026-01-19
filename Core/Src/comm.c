/*
 * comm.c (BOOTLOADER)
 * - TCP port 2001 receives a byte stream
 * - Assemble ETX OTA frames: [SOF][TYPE][LEN(LE16)][PAYLOAD][CRC32][EOF]
 * - Post complete frames to OTA worker (etx_ota_post_frame)
 *
 * IMPORTANT:
 *  - Do NOT call flash/OTA core from lwIP callbacks
 *  - This module is "fast path": just parse and post
 */

#include "comm.h"
#include "etx_ota_update.h"
#include <string.h>
#include <stdint.h>

/* Must match your protocol */
#ifndef ETX_OTA_SOF
#define ETX_OTA_SOF  0xAA
#endif
#ifndef ETX_OTA_EOF
#define ETX_OTA_EOF  0xBB
#endif

/* Stream assembly buffer (keep modest; frames are ~1033 bytes max for 1024 payload) */
#define COMM_RXBUF_SIZE  4096

static uint8_t s_rxbuf[COMM_RXBUF_SIZE];
static size_t  s_rxlen = 0;

static void drop_bytes(size_t n)
{
    if (n == 0 || s_rxlen == 0) return;
    if (n >= s_rxlen) { s_rxlen = 0; return; }
    memmove(s_rxbuf, s_rxbuf + n, s_rxlen - n);
    s_rxlen -= n;
}

/* Call from tcp_server_recv() after copying pbuf chain */
void comm_ota_rx_bytes(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    /* Append with simple overflow policy: drop oldest bytes (never drop in middle of a frame ideally) */
    if (len > (COMM_RXBUF_SIZE - s_rxlen)) {
        size_t need = len - (COMM_RXBUF_SIZE - s_rxlen);
        if (need >= s_rxlen) {
            s_rxlen = 0;
        } else {
            drop_bytes(need);
        }
    }

    memcpy(s_rxbuf + s_rxlen, data, len);
    s_rxlen += len;

    for (;;) {
        /* Find SOF */
        while (s_rxlen && s_rxbuf[0] != ETX_OTA_SOF) {
            drop_bytes(1);
        }
        if (s_rxlen < 4) return; /* need SOF+TYPE+LEN(2) */

        uint8_t  typ  = s_rxbuf[1];
        uint16_t dlen = (uint16_t)(s_rxbuf[2] | ((uint16_t)s_rxbuf[3] << 8));
        (void)typ;

        /* Total frame size */
        size_t total = 1u + 1u + 2u + (size_t)dlen + 4u + 1u;

        /* Sanity */
        if (total < (1u+1u+2u+4u+1u) || total > COMM_RXBUF_SIZE) {
            /* Bad length → resync by dropping SOF */
            drop_bytes(1);
            continue;
        }

        if (s_rxlen < total) {
            /* wait for more bytes */
            return;
        }

        if (s_rxbuf[total - 1] != ETX_OTA_EOF) {
            /* Bad EOF → resync by dropping SOF */
            drop_bytes(1);
            continue;
        }

        /* We have a full frame. Post to OTA worker. */
        etx_ota_post_frame(s_rxbuf, (uint16_t)total);

        /* Consume */
        drop_bytes(total);
    }
}
