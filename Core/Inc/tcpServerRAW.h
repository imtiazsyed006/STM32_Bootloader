/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  tcpServerRAW.h
  Author:     ControllersTech.com
  Updated:    26-Jul-2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_TCPSERVERRAW_H_
#define INC_TCPSERVERRAW_H_


#include "lwip/tcp.h"
#include "stm32f7xx_hal.h"

void tcp_server_init(void);

//struct tcp_pcb *g_port2000_pcb = NULL;  /* reply channel */
//struct tcp_pcb *g_port2001_pcb = NULL;  /* command channel */

/* Helper to send a short ASCII reply on port 2000 */
void send_reply_on_2000(const char *msg);



//extern uint8_t receivedData[100];
//extern uint8_t dataToSend;
//extern uint8_t interruptOccured;
/*  protocol states */
enum tcp_server_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

void pcb_send_bin(const uint8_t *data, size_t len, void *ctx);
void pcb_send_text(const char *msg, void *ctx);
 err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
 err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
 void tcp_server_error(void *arg, err_t err);
 err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
 err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
 void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
 err_t tcp_server_sent_data(void *arg, struct tcp_pcb *tpcb, u16_t len);
 void tcp_server_send_data(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
 void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
 void bl_send_text_2000_from_task(const char *s);

 void tcp_server_handle (struct tcp_pcb *tpcb, struct tcp_server_struct *es);
#endif /* INC_TCPSERVERRAW_H_ */
