#include "tcpserverRAW.h"
#include "comm.h"
#include "etx_ota_update.h"   /* <-- for etx_ota_set_resp_sender */

struct tcp_pcb *g_port2000_pcb = NULL;  /* reply channel */
struct tcp_pcb *g_port2001_pcb = NULL;  /* command channel */

void pcb_send_text(const char *msg, void *ctx) {
  struct tcp_pcb *pcb = (struct tcp_pcb *)ctx;
  if (!pcb || !msg) return;
  u16_t len = (u16_t)strlen(msg);
  if (!len) return;
  if (tcp_sndbuf(pcb) < len) return;
  if (tcp_write(pcb, msg, len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
    tcp_output(pcb);
  }
}

void pcb_send_bin(const uint8_t *data, size_t len, void *ctx) {
  struct tcp_pcb *pcb = (struct tcp_pcb *)ctx;
  if (!pcb || !data || !len) return;
  u16_t ulen = (u16_t)len;     // safe if your replies are small
  if (tcp_sndbuf(pcb) < ulen) return;
  if (tcp_write(pcb, data, ulen, TCP_WRITE_FLAG_COPY) == ERR_OK) {
    tcp_output(pcb);
  }
}

/* Sender used by OTA core to emit RESPONSE (ACK/NACK) on port 2000 */
static void ota_resp_tx_over_tcp(const uint8_t *data, size_t len, void *ctx) {
  struct tcp_pcb *pcb = (struct tcp_pcb*)ctx;
  if (pcb) pcb_send_bin(data, len, pcb);
}

void tcp_server_init(void)
{
    struct tcp_pcb *recv_pcb;
    struct tcp_pcb *send_pcb;
    err_t err;

    ip_addr_t myIPADDR;
    IP_ADDR4(&myIPADDR, 192, 168, 0, 25);

    // ----------- Command PCB (Port 2001, device receives) -----------
    recv_pcb = tcp_new();
    if (recv_pcb != NULL) {
        err = tcp_bind(recv_pcb, &myIPADDR, 2001);
        if (err == ERR_OK) {
            recv_pcb = tcp_listen(recv_pcb);
            tcp_accept(recv_pcb, tcp_server_accept);
        } else {
            memp_free(MEMP_TCP_PCB, recv_pcb);
        }
    }

    // ----------- Reply PCB (Port 2000, device sends ACK/echo) -----------
    send_pcb = tcp_new();
    if (send_pcb != NULL) {
        err = tcp_bind(send_pcb, &myIPADDR, 2000);
        if (err == ERR_OK) {
            send_pcb = tcp_listen(send_pcb);
            tcp_accept(send_pcb, tcp_server_accept);
        } else {
            memp_free(MEMP_TCP_PCB, send_pcb);
        }
    }
}

err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_server_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  tcp_setprio(newpcb, TCP_PRIO_MIN);

  es = (struct tcp_server_struct *)mem_malloc(sizeof(struct tcp_server_struct));
  if (es != NULL)
  {
    es->state   = ES_ACCEPTED;
    es->pcb     = newpcb;
    es->retries = 0;
    es->p       = NULL;

    /* Attach state to pcb */
    tcp_arg(newpcb, es);
    tcp_err(newpcb, tcp_server_error);

    if (newpcb->local_port == 2000) {
      /* Store the pcb we will use to SEND replies */
      g_port2000_pcb = newpcb;

      /* Register OTA ACK/NACK sender to use port 2000 */
      etx_ota_set_resp_sender(ota_resp_tx_over_tcp, (void*)g_port2000_pcb);

      /* We don't need to receive on 2000; it's a reply-only channel */
      tcp_recv(newpcb, NULL);
      tcp_sent(newpcb, tcp_server_sent);  /* optional */

    } else if (newpcb->local_port == 2001) {
      /* Store the pcb we will use to RECEIVE commands */
      g_port2001_pcb = newpcb;

      /* Receive commands here */
      tcp_recv(newpcb, tcp_server_recv);
      tcp_sent(newpcb, tcp_server_sent);  /* optional */
    }

    ret_err = ERR_OK;
  }
  else
  {
    tcp_server_connection_close(newpcb, es);
    ret_err = ERR_MEM;
  }
  return ret_err;
}

void send_reply_on_2000(const char *msg)
{
  if (!g_port2000_pcb || !msg) return;

  u16_t len = (u16_t)strlen(msg);
  if (len == 0) return;

  if (tcp_sndbuf(g_port2000_pcb) < len) {
    return;
  }

  if (tcp_write(g_port2000_pcb, msg, len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
    tcp_output(g_port2000_pcb);
  }
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_server_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);

  es = (struct tcp_server_struct *)arg;

  if (p == NULL)
  {
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       tcp_server_connection_close(tpcb, es);
    }
    else
    {
      tcp_sent(tpcb, tcp_server_sent);
      tcp_server_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }
  else if(err != ERR_OK)
  {
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED || es->state == ES_RECEIVED)
  {
      es->state = ES_RECEIVED;
      es->p = p;
      tcp_sent(tpcb, tcp_server_sent);
      tcp_server_handle(tpcb, es);
      ret_err = ERR_OK;
  }
  else if (es->state == ES_RECEIVED)
  {
    if(es->p == NULL)
    {
      es->p = p;
      tcp_server_handle(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;
      ptr = es->p;
      pbuf_chain(ptr,p);
    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  return ret_err;
}

void tcp_server_error(void *arg, err_t err)
{
  struct tcp_server_struct *es;
  LWIP_UNUSED_ARG(err);

  es = (struct tcp_server_struct *)arg;
  if (es != NULL)
  {
    mem_free(es);
  }
}

err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_server_struct *es;
  LWIP_UNUSED_ARG(len);

  es = (struct tcp_server_struct *)arg;
  es->retries = 0;

  if(es->p != NULL)
  {
    tcp_sent(tpcb, tcp_server_sent);
    tcp_server_send(tpcb, es);
  }
  else
  {
    if(es->state == ES_CLOSING)
      tcp_server_connection_close(tpcb, es);
  }
  return ERR_OK;
}

err_t tcp_server_sent_data(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_server_struct *es;
  LWIP_UNUSED_ARG(len);
  es = (struct tcp_server_struct *)arg;
  es->retries = 0;
  tcp_server_send_data(tpcb, es);
  return ERR_OK;
}

void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  while ((wr_err == ERR_OK) &&
         (es->p != NULL) &&
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    ptr = es->p;
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;

      es->p = ptr->next;

      if(es->p != NULL)
      {
        pbuf_ref(es->p);
      }

      do { freed = pbuf_free(ptr); }
      while(freed == 0);

      tcp_recved(tpcb, plen);
    }
    else if(wr_err == ERR_MEM)
    {
      es->p = ptr;
    }
    else
    {
      /* other problem ?? */
    }
  }
}

void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
  if (tpcb == g_port2000_pcb) {
    /* Clear responder so OTA falls back to UART if port 2000 drops */
    etx_ota_set_resp_sender(NULL, NULL);
    g_port2000_pcb = NULL;
  }
  if (tpcb == g_port2001_pcb) g_port2001_pcb = NULL;

  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);

  if (es != NULL) {
    mem_free(es);
  }
  tcp_close(tpcb);
}

void tcp_server_handle(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
    if (!es || !es->p) return;

    /* Command channel (host -> device) feeds the framed/text parser */
    if (tpcb && tpcb->local_port == 2001) {
        struct pbuf *q = es->p;
        while (q) {
            comm_handle_rx((const uint8_t*)q->payload, q->len,
                           pcb_send_text, pcb_send_bin, (void*)tpcb);
            q = q->next;
        }
    } else {
        /* optional: legacy text-only path */
        char in[128];
        u16_t to_copy = LWIP_MIN(es->p->tot_len, (u16_t)(sizeof(in) - 1));
        pbuf_copy_partial(es->p, in, to_copy, 0);
        in[to_copy] = '\0';
        comm_handle_line(in, pcb_send_text, pcb_send_bin, (void*)tpcb);
    }

    tcp_recved(tpcb, es->p->tot_len);
    pbuf_free(es->p);
    es->p = NULL;
}
