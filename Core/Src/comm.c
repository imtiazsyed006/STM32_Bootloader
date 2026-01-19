/*
 * comm.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Imtiaz
 */

#include "comm.h"

#include "etx_ota_update.h"     /* <-- use OTA types/APIs */
#include <inttypes.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

extern struct tcp_pcb *g_port2000_pcb;  /* reply channel */
extern struct tcp_pcb *g_port2001_pcb;  /* command channel */
extern uint8_t heartBeatMessage[4];
extern uint8_t tpdo[40];
extern uint8_t telemetryData[58];
extern uint8_t timer1Var;
extern uint8_t timer2Var;

/* RX assembly buffers for TCP stream on port 2001 */
static uint8_t s_rxbuf[4096];
static size_t  s_rxlen = 0;

static char    s_linebuf[256];
static size_t  s_linelen = 0;

/* forward */
static void comm_feed_text_bytes(const uint8_t *p, size_t n,
                                 comm_send_text_fn send_text,
                                 comm_send_bin_fn send_bin,
                                 void *ctx);

static int iequals(const char *a, const char *b) {
    for (;; a++, b++) {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return 0;
        if (ca == 0)  return 1;
    }
}

static int starts_with_ci(const char *s, const char *prefix) {
    while (*prefix) {
        unsigned char cs = (unsigned char)*s++;
        unsigned char cp = (unsigned char)*prefix++;
        if (tolower(cs) != tolower(cp)) return 0;
    }
    return 1;
}

/* skip leading spaces */
static const char* skip_ws(const char* p) {
    while (*p == ' ' || *p == '\t') ++p;
    return p;
}


/* -------------------------------------------------------------------------- */
/* Command handler (text line commands)                                       */
/* -------------------------------------------------------------------------- */

void comm_handle_line(const char *line,
                      comm_send_text_fn send_text,
                      comm_send_bin_fn send_bin,
                      void *ctx)
{
    if (!line || !send_text) return;

    /* simple echo of AA..BB if user pasted a hex-y line */
    {
        size_t len = strlen(line);
        if (len >= 2 &&
            ((uint8_t)line[0] == ETX_OTA_SOF) &&
            ((uint8_t)line[len - 1] == ETX_OTA_EOF))
        {
            if (send_bin && g_port2000_pcb) {
                send_bin((const uint8_t*)line, len, g_port2000_pcb);
            } else {
                send_text("ERR:Port2000 not ready\n", ctx);
            }
            return;
        }
    }

    if (iequals(line, "Hello")) {
        send_text("Hi\n", ctx);
        return;
    }
    if (iequals(line, "HeartBeat") || iequals(line, "Heartbeat")) {
        send_text("Alive\n", ctx);
        TIM1->CNT = 0;
        return;
    }


    if (strncmp(line, "BIN:", 4) == 0) {
        uint8_t buf[64];
        size_t blen = 0;
        const char *p = line + 4;
        while (*p && *(p+1) && blen < sizeof(buf)) {
            char h[3] = { p[0], p[1], 0 };
            unsigned v = 0;
            if (sscanf(h, "%2x", &v) != 1) break;
            buf[blen++] = (uint8_t)v;
            p += 2;
        }
        if (blen && send_bin) send_bin(buf, blen, ctx);
        return;
    }


    send_text("ERR:Unknown command\n", ctx);
}


/* -------------------------------------------------------------------------- */
/* Streaming framed parser for TCP port 2001                                  */
/* -------------------------------------------------------------------------- */
void comm_handle_rx(const uint8_t *data, size_t len,
                    comm_send_text_fn send_text,
                    comm_send_bin_fn  send_bin,
                    void *ctx)
{
    if (!data || !len) return;

    /* Append to RX buffer (simple backpressure) */
    if (len > sizeof(s_rxbuf) - s_rxlen) {
        size_t need = len - (sizeof(s_rxbuf) - s_rxlen);
        if (need >= s_rxlen) {
            s_rxlen = 0;  /* drop all if overrun */
        } else {
            memmove(s_rxbuf, s_rxbuf + need, s_rxlen - need);
            s_rxlen -= need;
        }
    }
    memcpy(s_rxbuf + s_rxlen, data, len);
    s_rxlen += len;

    for (;;) {
        /* 1) Flush leading non-SOF as text bytes */
        while (s_rxlen && s_rxbuf[0] != ETX_OTA_SOF) {
            size_t i = 0;
            while (i < s_rxlen && s_rxbuf[i] != ETX_OTA_SOF) i++;
            comm_feed_text_bytes(s_rxbuf, i, send_text, send_bin, ctx);
            memmove(s_rxbuf, s_rxbuf + i, s_rxlen - i);
            s_rxlen -= i;
            if (!s_rxlen) return;
        }

        /* Need at least SOF + TYPE + LEN(2) */
        if (s_rxlen < 4) return;

        uint8_t  typ   = s_rxbuf[1];
        uint16_t dlen  = (uint16_t)(s_rxbuf[2] | ((uint16_t)s_rxbuf[3] << 8));
        size_t   total = 1 + 1 + 2 + dlen + 4 + 1;
        (void)typ;

        if (total > sizeof(s_rxbuf)) {
            /* bogus length; drop SOF and resync */
            memmove(s_rxbuf, s_rxbuf + 1, s_rxlen - 1);
            s_rxlen -= 1;
            continue;
        }
        if (s_rxlen < total) {
            /* wait for more bytes */
            return;
        }

        uint8_t *frame = s_rxbuf;
        if (frame[total - 1] != ETX_OTA_EOF) {
            /* bad EOF: drop one byte and resync */
            memmove(s_rxbuf, s_rxbuf + 1, s_rxlen - 1);
            s_rxlen -= 1;
            continue;
        }

        /* Hand to OTA core (sends ACK/NACK via etx_ota_send_resp path) */
        ETX_OTA_EX_ r = etx_ota_feed_frame(frame, (uint16_t)total);

        /* If accepted: also echo the frame back on port 2000, as requested */
        if (r == ETX_OTA_EX_OK) {
            if (send_bin && g_port2000_pcb) {
                send_bin(frame, total, g_port2000_pcb);
            } else if (send_text) {
                send_text("ERR:Port2000 not ready\n", ctx);
            }
        }

        /* consume frame and continue parsing */
        memmove(s_rxbuf, s_rxbuf + total, s_rxlen - total);
        s_rxlen -= total;
    }
}

/* accumulate ASCII until newline, then call comm_handle_line() */
static void comm_feed_text_bytes(const uint8_t *p, size_t n,
                                 comm_send_text_fn send_text,
                                 comm_send_bin_fn send_bin,
                                 void *ctx)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t c = p[i];
        if (c == '\r') continue;
        if (c == '\n') {
            if (s_linelen < sizeof(s_linebuf)) s_linebuf[s_linelen] = '\0';
            else s_linebuf[sizeof(s_linebuf) - 1] = '\0';
            if (s_linelen) {
                comm_handle_line(s_linebuf, send_text, send_bin, ctx);
            }
            s_linelen = 0;
            continue;
        }
        if (s_linelen < sizeof(s_linebuf) - 1) {
            s_linebuf[s_linelen++] = (char)c;
        }
    }
}
