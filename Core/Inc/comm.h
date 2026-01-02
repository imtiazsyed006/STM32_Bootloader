/*
 * comm.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Imtiaz
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_
#include "stm32f7xx_hal.h"


// Callback types used by the parser to send responses
typedef void (*comm_send_text_fn)(const char *msg, void *ctx);
typedef void (*comm_send_bin_fn)(const uint8_t *data, size_t len, void *ctx);

// Parse and handle a single command line (already trimmed of CR/LF).
// Implement all your app logic here. Use the supplied callbacks to reply.
void comm_handle_line(const char *line,
                      comm_send_text_fn send_text,
                      comm_send_bin_fn send_bin,
                      void *ctx);
void comm_handle_rx(const uint8_t *data, size_t len,
                    comm_send_text_fn send_text,
                    comm_send_bin_fn send_bin,
                    void *ctx);
#endif /* INC_COMM_H_ */
