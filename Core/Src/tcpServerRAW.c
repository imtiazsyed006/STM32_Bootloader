/*
 * tcpserverRAW.c (BOOTLOADER)
 * Port 2001: RX OTA frames from host
 * Port 2000: TX ACK/NACK responses to host (reply-only)
 *
 * IMPORTANT:
 *  - Do not process OTA/flash inside tcp_recv callback
 *  - Just copy bytes -> comm_ota_rx_bytes() -> free pbuf
 */

#include "tcpserverRAW.h"

#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/mem.h"
#include "lwip/ip_addr.h"
#include "lwip/opt.h"

#include "comm.h"
#include "etx_ota_update.h"

#include <string.h>

struct tcp_pcb *g_port2000_pcb = NULL;  /* reply channel (device->host) */
struct tcp_pcb *g_port2001_pcb = NULL;  /* command/data channel (host->device) */

/* ---------- Reply sender (lwIP-safe) ----------
 * Called by OTA worker task.
 * Schedules tcp_write() onto tcpip_thread via tcpip_callback().
 */
typedef struct {
    struct tcp_pcb *pcb;
    uint16_t len;
    uint8_t  data[16]; /* RESPONSE frame is small (typically 10 bytes) */
} resp_send_req_t;
extern struct tcp_pcb *g_port2000_pcb;

typedef struct {
  struct tcp_pcb *pcb;
  uint16_t len;
  char     msg[96];
} txt_req_t;

static void do_send_txt(void *arg)
{
  txt_req_t *r = (txt_req_t*)arg;
  if (!r) return;

  if (r->pcb && tcp_sndbuf(r->pcb) >= r->len) {
    if (tcp_write(r->pcb, r->msg, r->len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
      tcp_output(r->pcb);
    }
  }
  mem_free(r);
}

void bl_send_text_2000_from_task(const char *s)
{
  if (!s || !g_port2000_pcb) return;

  size_t n = strlen(s);
  if (n == 0) return;
  if (n > 95) n = 95;

  txt_req_t *r = (txt_req_t*)mem_malloc(sizeof(txt_req_t));
  if (!r) return;

  r->pcb = g_port2000_pcb;
  r->len = (uint16_t)n;
  memcpy(r->msg, s, n);
  r->msg[n] = '\0';

  tcpip_callback(do_send_txt, r);
}
static void do_send_resp(void *arg)
{
    resp_send_req_t *r = (resp_send_req_t*)arg;
    if (!r) return;

    if (r->pcb) {
        if (tcp_sndbuf(r->pcb) >= r->len) {
            if (tcp_write(r->pcb, r->data, r->len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
                tcp_output(r->pcb);
            }
        }
    }
    mem_free(r);
}

void ota_resp_tx_lwip(const uint8_t *data, uint16_t len, void *ctx)
{
    struct tcp_pcb *pcb = (struct tcp_pcb*)ctx;
    if (!data || len == 0 || !pcb) return;

    resp_send_req_t *r = (resp_send_req_t*)mem_malloc(sizeof(resp_send_req_t));
    if (!r) return;

    r->pcb = pcb;
    r->len = (len > sizeof(r->data)) ? (uint16_t)sizeof(r->data) : len;
    memcpy(r->data, data, r->len);

    tcpip_callback(do_send_resp, r);
}

/* ---------- Connection bookkeeping ---------- */
 err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv2001(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_server_close(struct tcp_pcb *tpcb);

/* NOTE: tcp_err callback arg is the value set by tcp_arg(). */
static void tcp_server_err(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(err);
    /* We set tcp_arg(pcb, NULL), so arg may be NULL.
       IMPORTANT: pcb is already freed here; DO NOT call any tcp_* API. */
    LWIP_UNUSED_ARG(arg);

    /* Just clear globals defensively (if you track by pointer elsewhere, do it there). */
    g_port2000_pcb = NULL;
    g_port2001_pcb = NULL;
}

void tcp_server_init(void)
{
    ip_addr_t ip;
    IP_ADDR4(&ip, 192, 168, 0, 25);

    /* Listen on 2001 (RX from host) */
    struct tcp_pcb *pcb2001 = tcp_new();
    if (pcb2001) {
        if (tcp_bind(pcb2001, &ip, 2001) == ERR_OK) {
            pcb2001 = tcp_listen(pcb2001);
            tcp_accept(pcb2001, tcp_server_accept);
        } else {
            tcp_close(pcb2001);
        }
    }

    /* Listen on 2000 (TX replies to host) */
    struct tcp_pcb *pcb2000 = tcp_new();
    if (pcb2000) {
        if (tcp_bind(pcb2000, &ip, 2000) == ERR_OK) {
            pcb2000 = tcp_listen(pcb2000);
            tcp_accept(pcb2000, tcp_server_accept);
        } else {
            tcp_close(pcb2000);
        }
    }
}

 err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    tcp_setprio(newpcb, TCP_PRIO_MIN);
    tcp_nagle_disable(newpcb);

    /* VERY IMPORTANT: set err callback (pcb can die unexpectedly) */
    tcp_err(newpcb, tcp_server_err);

    if (newpcb->local_port == 2000) {
        g_port2000_pcb = newpcb;

        /* reply-only: do not receive on 2000 */
        tcp_recv(newpcb, NULL);

        /* tell OTA core how to send ACK/NACK safely */
        etx_ota_set_resp_sender(ota_resp_tx_lwip, g_port2000_pcb);

        const char *hello = "PORT2000 READY\r\n";
        tcp_write(newpcb, hello, (u16_t)strlen(hello), TCP_WRITE_FLAG_COPY);
        tcp_output(newpcb);
        return ERR_OK;
    }

    if (newpcb->local_port == 2001) {
        g_port2001_pcb = newpcb;

        /* RX path */
        tcp_recv(newpcb, tcp_server_recv2001);
        return ERR_OK;
    }

    tcp_server_close(newpcb);
    return ERR_VAL;
}

/* Fast path receive: copy bytes -> comm -> free */
static err_t tcp_server_recv2001(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    LWIP_UNUSED_ARG(arg);

    if (p == NULL) {
        tcp_server_close(tpcb);
        return ERR_OK;
    }

    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    /* Pass pbuf chain to stream parser */
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        if (q->len && q->payload) {
            comm_ota_rx_bytes((const uint8_t*)q->payload, (size_t)q->len);
        }
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static void tcp_server_close(struct tcp_pcb *tpcb)
{
    if (!tpcb) return;

    if (tpcb == g_port2000_pcb) g_port2000_pcb = NULL;
    if (tpcb == g_port2001_pcb) g_port2001_pcb = NULL;

    tcp_arg(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    tcp_close(tpcb);
}
