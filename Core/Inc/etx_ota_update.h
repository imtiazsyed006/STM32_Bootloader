/*
 * etx_ota_update.h
 * Minimal UART + TCP-capable OTA interface
 */

#ifndef INC_ETX_OTA_UPDATE_H_
#define INC_ETX_OTA_UPDATE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "main.h"

/* Framing + status */
#define ETX_OTA_SOF  0xAA
#define ETX_OTA_EOF  0xBB
#define ETX_OTA_ACK  0x00
#define ETX_OTA_NACK 0x01

/* Flash map (unchanged) */
#define ETX_APP_FLASH_ADDR        0x08040000   /* Application address */
#define ETX_APP_SLOT0_FLASH_ADDR  0x080C0000   /* Slot 0 */
#define ETX_APP_SLOT1_FLASH_ADDR  0x08140000   /* Slot 1 */
#define ETX_CONFIG_FLASH_ADDR     0x08020000   /* Config */

#define ETX_NO_OF_SLOTS           2
#define ETX_SLOT_MAX_SIZE        (512 * 1024)

#define ETX_OTA_DATA_MAX_SIZE     (1024)
#define ETX_OTA_DATA_OVERHEAD     (9)
#define ETX_OTA_PACKET_MAX_SIZE   (ETX_OTA_DATA_MAX_SIZE + ETX_OTA_DATA_OVERHEAD)

/* Reboot reason */
#define ETX_FIRST_TIME_BOOT       ( 0xFFFFFFFF )
#define ETX_NORMAL_BOOT           ( 0xBEEFFEED )
#define ETX_OTA_REQUEST           ( 0xDEADBEEF )
#define ETX_LOAD_PREV_APP         ( 0xFACEFADE )

/* Return codes */
typedef enum
{
  ETX_OTA_EX_OK  = 0,
  ETX_OTA_EX_ERR = 1,
} ETX_OTA_EX_;

/* OTA process state */
typedef enum
{
  ETX_OTA_STATE_IDLE   = 0,
  ETX_OTA_STATE_START  = 1,
  ETX_OTA_STATE_HEADER = 2,
  ETX_OTA_STATE_DATA   = 3,
  ETX_OTA_STATE_END    = 4,
} ETX_OTA_STATE_;

/* Packet type */
typedef enum
{
  ETX_OTA_PACKET_TYPE_CMD       = 0,
  ETX_OTA_PACKET_TYPE_DATA      = 1,
  ETX_OTA_PACKET_TYPE_HEADER    = 2,
  ETX_OTA_PACKET_TYPE_RESPONSE  = 3,
} ETX_OTA_PACKET_TYPE_;

/* OTA Commands */
typedef enum
{
  ETX_OTA_CMD_START = 0,
  ETX_OTA_CMD_END   = 1,
  ETX_OTA_CMD_ABORT = 2,
} ETX_OTA_CMD_;

/* Slot table */
typedef struct
{
    uint8_t  is_this_slot_not_valid;
    uint8_t  is_this_slot_active;
    uint8_t  should_we_run_this_fw;
    uint32_t fw_size;
    uint32_t fw_crc;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
} __attribute__((packed)) ETX_SLOT_;

/* General configuration */
typedef struct
{
    uint32_t  reboot_cause;
    ETX_SLOT_ slot_table[ETX_NO_OF_SLOTS];
} __attribute__((packed)) ETX_GNRL_CFG_;

/* OTA meta info */
typedef struct
{
  uint32_t package_size;
  uint32_t package_crc;
  uint32_t reserved1;
  uint32_t reserved2;
} __attribute__((packed)) meta_info;

/* Wire formats */
typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   cmd;
  uint32_t  crc;
  uint8_t   eof;
} __attribute__((packed)) ETX_OTA_COMMAND_;

typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  meta_info   meta_data;
  uint32_t    crc;
  uint8_t     eof;
} __attribute__((packed)) ETX_OTA_HEADER_;

typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  uint8_t     *data;   /* points into the same buffer; payload starts at offset 4 */
} __attribute__((packed)) ETX_OTA_DATA_;

typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   status;
  uint32_t  crc;
  uint8_t   eof;
} __attribute__((packed)) ETX_OTA_RESP_;

/* Transport abstraction for RESPONSE frames (ACK/NACK) */
/* NOTE: len is uint16_t to match the implementation */
typedef void (*etx_tx_fn_t)(const uint8_t *data, uint16_t len, void *ctx);

/* APIs */
ETX_OTA_EX_ etx_ota_download_and_flash(void);   /* UART blocking path */
void        load_new_app(void);

/* Register how to send RESPONSE frames over TCP (or NULL to fallback to UART) */
void        etx_ota_set_resp_sender(etx_tx_fn_t fn, void *ctx);

/* Feed a complete framed packet (SOF..EOF) from TCP; returns ETX_OTA_EX_OK on accept */
ETX_OTA_EX_ etx_ota_feed_frame(const uint8_t *frame, uint16_t len);
void etx_ota_send_text_raw(const char *s, uint16_t n);
void etx_ota_logf(const char *fmt, ...);
void OTA_LOGF(const char *fmt, ...);
void goto_application(void);
void etx_ota_worker_start(void);
//#define OTA_LOGF(...)  do { etx_ota_logf(__VA_ARGS__); } while (0)
#endif /* INC_ETX_OTA_UPDATE_H_ */
