///*
// * etx_ota_update.c
// * UART3 + TCP-capable OTA flow: receive -> write slot -> mark -> load_new_app()
// *
// * DEBUG VERSION:
// *  - All logs forced to UART (so port 2000 stays clean for ACK/NACK)
// *  - Extra logging added for tracing RTOS hangs
// */
//
//#include <stdio.h>
//#include <string.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include "etx_ota_update.h"
//#include "main.h"
//#include <stdarg.h>
//
//static uint8_t Rx_Buffer[ ETX_OTA_PACKET_MAX_SIZE ];
//
//static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;
//
//static uint32_t ota_fw_total_size;
//static uint32_t ota_fw_crc;
//static uint32_t ota_fw_received_size;
//static uint8_t  slot_num_to_write;
//
//extern uint8_t updateComplete;
//extern CRC_HandleTypeDef  hcrc;
//extern UART_HandleTypeDef huart3;
//
//ETX_GNRL_CFG_ *cfg_flash = (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR);
//
//#ifndef ETX_TX_FN_T_DEFINED
//typedef void (*etx_tx_fn_t)(const uint8_t *data, uint16_t len, void *ctx);
//#define ETX_TX_FN_T_DEFINED 1
//#endif
//static etx_tx_fn_t s_resp_tx  = NULL;
//static void       *s_resp_ctx = NULL;
//
//static uint16_t      etx_receive_chunk(uint8_t *buf, uint16_t max_len);
//static ETX_OTA_EX_   etx_process_data(uint8_t *buf, uint16_t len);
//static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num,
//                                            uint8_t *data,
//                                            uint16_t data_len,
//                                            bool is_first_block);
//static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len);
//static uint8_t       get_available_slot_number(void);
//static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg);
//
//static ETX_OTA_EX_   finalize_staged_image_and_mark(void);
//
//static const char* ota_state_str(ETX_OTA_STATE_ s)
//{
//    switch (s) {
//    case ETX_OTA_STATE_IDLE:   return "IDLE";
//    case ETX_OTA_STATE_START:  return "START";
//    case ETX_OTA_STATE_HEADER: return "HEADER";
//    case ETX_OTA_STATE_DATA:   return "DATA";
//    case ETX_OTA_STATE_END:    return "END";
//    default: return "?";
//    }
//}
//
//static uint32_t crc32_stm32_bytes(const uint8_t *data, uint32_t len)
//{
//    uint32_t crc = 0xFFFFFFFFu;
//    for (uint32_t i = 0; i < len; ++i) {
//        crc ^= ((uint32_t)data[i] << 24);
//        for (uint8_t b = 0; b < 8; ++b) {
//            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
//        }
//    }
//    return crc;
//}
//
///* -------------------- FORCED UART LOGGING -------------------- */
//void etx_ota_send_text_raw(const char *s, uint16_t n)
//{
//    if (!s || n == 0) return;
//
//    /* IMPORTANT: force all logs to UART so port 2000 stays clean for ACK/NACK */
//    HAL_UART_Transmit(&huart3, (uint8_t*)s, n, HAL_MAX_DELAY);
//}
//
//void etx_ota_logf(const char *fmt, ...)
//{
//    char buf[256];
//    va_list ap;
//    va_start(ap, fmt);
//    int n = vsnprintf(buf, sizeof buf, fmt, ap);
//    va_end(ap);
//    if (n < 0) return;
//    if ((size_t)n >= sizeof buf) n = sizeof(buf) - 1;
//    etx_ota_send_text_raw(buf, (uint16_t)n);
//}
//
//static void ota_reset_session(void)
//{
//    OTA_LOGF("[OTA] reset_session()\r\n");
//    ota_fw_total_size    = 0u;
//    ota_fw_received_size = 0u;
//    ota_fw_crc           = 0u;
//    slot_num_to_write    = 0xFFu;
//    ota_state            = ETX_OTA_STATE_START;
//}
//
//void etx_ota_set_resp_sender(etx_tx_fn_t fn, void *ctx)
//{
//    s_resp_tx  = fn;
//    s_resp_ctx = ctx;
//    OTA_LOGF("[OTA] resp_sender set: fn=%p ctx=%p\r\n", fn, ctx);
//}
//
///* ACK/NACK response (this MUST remain TCP-capable) */
//static void etx_ota_send_resp(uint8_t type)
//{
//    ETX_OTA_RESP_ rsp =
//    {
//        .sof         = ETX_OTA_SOF,
//        .packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
//        .data_len    = 1u,
//        .status      = type,
//        .eof         = ETX_OTA_EOF
//    };
//
//    rsp.crc = crc32_stm32_bytes(&rsp.status, 1u);
//
//    if (s_resp_tx) {
//        s_resp_tx((const uint8_t *)&rsp, (uint16_t)sizeof(rsp), s_resp_ctx);
//    } else {
//        HAL_UART_Transmit(&huart3, (uint8_t *)&rsp, sizeof(rsp), HAL_MAX_DELAY);
//    }
//}
//
//ETX_OTA_EX_ etx_ota_feed_frame(const uint8_t *frame, uint16_t len)
//{
//    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;
//
//    do {
//        if (!frame || len < (1 + 1 + 2 + 4 + 1)) {
//            OTA_LOGF("[OTA] feed_frame: bad args frame=%p len=%u\r\n", frame, len);
//            break;
//        }
//
//        if (frame[0] != ETX_OTA_SOF || frame[len-1] != ETX_OTA_EOF) {
//            OTA_LOGF("[OTA] feed_frame: bad SOF/EOF (SOF=0x%02X EOF=0x%02X) len=%u\r\n",
//                     frame[0], frame[len-1], len);
//            break;
//        }
//
//        uint8_t  typ   = frame[1];
//        uint16_t dlen  = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
//        uint32_t need  = (uint32_t)(1 + 1 + 2) + dlen + 4 + 1;
//
//        OTA_LOGF("[OTA] RX frame: type=%u dlen=%u total=%u state=%s\r\n",
//                 (unsigned)typ, (unsigned)dlen, (unsigned)len, ota_state_str(ota_state));
//
//        if (len != need) {
//            OTA_LOGF("[OTA] feed_frame: length mismatch got=%u need=%lu\r\n",
//                     (unsigned)len, (unsigned long)need);
//            break;
//        }
//
//        if (len > ETX_OTA_PACKET_MAX_SIZE) {
//            OTA_LOGF("[OTA] feed_frame: oversize packet len=%u max=%u\r\n",
//                     (unsigned)len, (unsigned)ETX_OTA_PACKET_MAX_SIZE);
//            etx_ota_send_resp(ETX_OTA_NACK);
//            break;
//        }
//
//        const uint8_t* payload = &frame[4];
//        uint32_t rec_crc =  (uint32_t)payload[dlen]
//                          | ((uint32_t)payload[dlen + 1] << 8)
//                          | ((uint32_t)payload[dlen + 2] << 16)
//                          | ((uint32_t)payload[dlen + 3] << 24);
//        uint32_t cal_crc = crc32_stm32_bytes(payload, dlen);
//
//        if (typ == ETX_OTA_PACKET_TYPE_HEADER) {
//            OTA_LOGF("[OTA] HDR bytes: %02X %02X %02X %02X %02X %02X %02X %02X ...\r\n",
//                     frame[0], frame[1], frame[2], frame[3],
//                     frame[4], frame[5], frame[6], frame[7]);
//        }
//
//        if (cal_crc != rec_crc) {
//            OTA_LOGF("[OTA] CRC mismatch: calc=0x%08lX rec=0x%08lX typ=%u dlen=%u\r\n",
//                     (unsigned long)cal_crc, (unsigned long)rec_crc,
//                     (unsigned)typ, (unsigned)dlen);
//            etx_ota_send_resp(ETX_OTA_NACK);
//            break;
//        }
//
//        if (ota_state == ETX_OTA_STATE_IDLE) {
//            if (typ == ETX_OTA_PACKET_TYPE_CMD && dlen == 1 && payload[0] == ETX_OTA_CMD_START) {
//                OTA_LOGF("[OTA] IDLE->START by CMD_START\r\n");
//                ota_reset_session();
//            } else {
//                OTA_LOGF("[OTA] In IDLE but got non-START -> NACK\r\n");
//                etx_ota_send_resp(ETX_OTA_NACK);
//                break;
//            }
//        }
//
//        memcpy(Rx_Buffer, frame, len);
//        ret = etx_process_data(Rx_Buffer, (uint16_t)len);
//
//        OTA_LOGF("[OTA] process ret=%u -> RESP=%s\r\n",
//                 (unsigned)ret, (ret == ETX_OTA_EX_OK) ? "ACK" : "NACK");
//
//        etx_ota_send_resp((ret == ETX_OTA_EX_OK) ? ETX_OTA_ACK : ETX_OTA_NACK);
//    } while (false);
//
//    return ret;
//}
//
///* UART blocking path (unchanged) */
//ETX_OTA_EX_ etx_ota_download_and_flash(void)
//{
//    ETX_OTA_EX_ ret  = ETX_OTA_EX_OK;
//    uint16_t    len;
//
//    ota_fw_total_size    = 0u;
//    ota_fw_received_size = 0u;
//    ota_fw_crc           = 0u;
//    ota_state            = ETX_OTA_STATE_START;
//    slot_num_to_write    = 0xFFu;
//
//    do {
//        memset(Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE);
//        len = etx_receive_chunk(Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE);
//
//        if (len != 0u) {
//            ret = etx_process_data(Rx_Buffer, len);
//        } else {
//            ret = ETX_OTA_EX_ERR;
//        }
//
//        if (ret != ETX_OTA_EX_OK) {
//            OTA_LOGF("[OTA] UART path NACK\r\n");
//            etx_ota_send_resp(ETX_OTA_NACK);
//            break;
//        } else {
//            etx_ota_send_resp(ETX_OTA_ACK);
//        }
//    } while (ota_state != ETX_OTA_STATE_IDLE);
//
//    return ret;
//}
//
//static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len)
//{
//    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;
//
//    do {
//        if ((buf == NULL) || (len == 0u)) {
//            OTA_LOGF("[OTA] process_data: bad args buf=%p len=%u\r\n", buf, len);
//            break;
//        }
//
//        OTA_LOGF("[OTA] process_data: state=%s len=%u\r\n", ota_state_str(ota_state), (unsigned)len);
//
//        ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*)buf;
//        if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD && cmd->cmd == ETX_OTA_CMD_ABORT) {
//            OTA_LOGF("[OTA] ABORT received\r\n");
//            break;
//        }
//
//        switch (ota_state) {
//
//        case ETX_OTA_STATE_IDLE:
//            ret = ETX_OTA_EX_OK;
//            break;
//
//        case ETX_OTA_STATE_START:
//        {
//            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
//            OTA_LOGF("[OTA] START: pkt_type=%u cmd=%u\r\n", (unsigned)c->packet_type, (unsigned)c->cmd);
//            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_START) {
//                OTA_LOGF("[OTA] Started...\r\n");
//                ota_state = ETX_OTA_STATE_HEADER;
//                ret = ETX_OTA_EX_OK;
//            }
//        } break;
//
//        case ETX_OTA_STATE_HEADER:
//        {
//            ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*)buf;
//            OTA_LOGF("[OTA] HEADER: pkt_type=%u dlen=%u\r\n",
//                     (unsigned)header->packet_type, (unsigned)header->data_len);
//
//            if (header->packet_type == ETX_OTA_PACKET_TYPE_HEADER) {
//                ota_fw_total_size = header->meta_data.package_size;
//                ota_fw_crc        = header->meta_data.package_crc;
//
//                OTA_LOGF("[OTA] HEADER meta: size=%lu crc=0x%08lX\r\n",
//                         (unsigned long)ota_fw_total_size,
//                         (unsigned long)ota_fw_crc);
//
//                ETX_GNRL_CFG_ cfg;
//                memcpy(&cfg, cfg_flash, sizeof(cfg));
//                OTA_LOGF("[OTA] CFG: s0 act=%u inv=%u run=%u | s1 act=%u inv=%u run=%u\r\n",
//                         cfg.slot_table[0].is_this_slot_active,
//                         cfg.slot_table[0].is_this_slot_not_valid,
//                         cfg.slot_table[0].should_we_run_this_fw,
//                         cfg.slot_table[1].is_this_slot_active,
//                         cfg.slot_table[1].is_this_slot_not_valid,
//                         cfg.slot_table[1].should_we_run_this_fw);
//
//                slot_num_to_write = get_available_slot_number();
//                OTA_LOGF("[OTA] slot_num_to_write=%u\r\n", (unsigned)slot_num_to_write);
//
//                if (slot_num_to_write != 0xFFu) {
//                    ota_state = ETX_OTA_STATE_DATA;
//                    OTA_LOGF("[OTA] HEADER OK -> state=DATA\r\n");
//                    ret = ETX_OTA_EX_OK;
//                } else {
//                    OTA_LOGF("[OTA] No available slot -> ERR\r\n");
//                }
//            }
//        } break;
//
//        case ETX_OTA_STATE_DATA:
//        {
//            ETX_OTA_DATA_ *data = (ETX_OTA_DATA_*)buf;
//            uint16_t dlen = data->data_len;
//
//            OTA_LOGF("[OTA] DATA: pkt_type=%u dlen=%u rx_total=%lu/%lu\r\n",
//                     (unsigned)data->packet_type,
//                     (unsigned)dlen,
//                     (unsigned long)ota_fw_received_size,
//                     (unsigned long)ota_fw_total_size);
//
//            if (data->packet_type == ETX_OTA_PACKET_TYPE_DATA) {
//                bool first_block = (ota_fw_received_size == 0);
//
//                if (first_block) {
//                    OTA_LOGF("[OTA] First DATA block\r\n");
//                    ETX_GNRL_CFG_ cfg;
//                    memcpy(&cfg, cfg_flash, sizeof(cfg));
//                    cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;
//                    HAL_StatusTypeDef rcfg = write_cfg_to_flash(&cfg);
//                    OTA_LOGF("[OTA] write_cfg_to_flash(first_block) rc=%d\r\n", (int)rcfg);
//                    if (rcfg != HAL_OK) break;
//                }
//
//                HAL_StatusTypeDef ex = write_data_to_slot(slot_num_to_write, buf + 4, dlen, first_block);
//                OTA_LOGF("[OTA] write_data_to_slot rc=%d new_rx=%lu\r\n",
//                         (int)ex, (unsigned long)ota_fw_received_size);
//
//                if (ex == HAL_OK) {
//                    if (ota_fw_received_size >= ota_fw_total_size) {
//                        OTA_LOGF("[OTA] All bytes received -> finalize\r\n");
//                        ret = finalize_staged_image_and_mark();
//                        OTA_LOGF("[OTA] finalize ret=%u\r\n", (unsigned)ret);
//                        if (ret == ETX_OTA_EX_OK) {
//                            ota_state = ETX_OTA_STATE_IDLE;
//                            updateComplete = 1;
//                            OTA_LOGF("[OTA] DONE -> state=IDLE updateComplete=1\r\n");
//                        }
//                        break;
//                    }
//                    ret = ETX_OTA_EX_OK;
//                } else {
//                    OTA_LOGF("[OTA] DATA write error!\r\n");
//                }
//            }
//        } break;
//
//        case ETX_OTA_STATE_END:
//        {
//            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
//            OTA_LOGF("[OTA] END: pkt_type=%u cmd=%u\r\n", (unsigned)c->packet_type, (unsigned)c->cmd);
//            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_END) {
//                OTA_LOGF("[OTA] END received -> finalize\r\n");
//                ret = finalize_staged_image_and_mark();
//                if (ret == ETX_OTA_EX_OK) {
//                    ota_state = ETX_OTA_STATE_IDLE;
//                    updateComplete = 1;
//                }
//            }
//        } break;
//
//        default:
//            OTA_LOGF("[OTA] default: unknown state\r\n");
//            ret = ETX_OTA_EX_ERR;
//            break;
//        }
//
//    } while (false);
//
//    return ret;
//}
//
///* ----------- the rest of your functions are unchanged ----------- */
//
//static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num,
//                                            uint8_t *data,
//                                            uint16_t data_len,
//                                            bool is_first_block)
//{
//    HAL_StatusTypeDef ret;
//
//    do {
//        if (slot_num >= ETX_NO_OF_SLOTS) { ret = HAL_ERROR; break; }
//
//        ret = HAL_FLASH_Unlock();
//        if (ret != HAL_OK) break;
//
//        if (is_first_block) {
//            OTA_LOGF("[OTA] Erasing Slot %d...\r\n", slot_num);
//            FLASH_EraseInitTypeDef EraseInitStruct;
//            uint32_t SectorError;
//
//            EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
//            EraseInitStruct.Sector       = (slot_num == 0) ? FLASH_SECTOR_7 : FLASH_SECTOR_9;
//            EraseInitStruct.NbSectors    = 2;
//            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//
//            ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
//            if (ret != HAL_OK) { OTA_LOGF("[OTA] Erase error\r\n"); break; }
//        }
//
//        uint32_t flash_addr = (slot_num == 0) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;
//
//        for (uint32_t i = 0; i < (uint32_t)data_len; i++) {
//            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
//                                    (flash_addr + ota_fw_received_size),
//                                    data[i]);
//            if (ret == HAL_OK) {
//                ota_fw_received_size += 1u;
//            } else {
//                OTA_LOGF("[OTA] Write error\r\n");
//                break;
//            }
//        }
//
//        if (ret != HAL_OK) break;
//
//        ret = HAL_FLASH_Lock();
//        if (ret != HAL_OK) break;
//
//    } while (false);
//
//    return ret;
//}
/*
 * etx_ota_update.c
 * UART3 + TCP-capable OTA flow: receive -> write slot -> mark -> load_new_app()
 *
 * DEBUG VERSION:
 *  - All logs forced to UART (so port 2000 stays clean for ACK/NACK)
 *  - Extra logging added for tracing RTOS hangs
 *
 * FIXES ADDED:
 *  - DATA dlen parsed from raw bytes (avoid struct packing issues)
 *  - Reject DATA overflow beyond expected total_size (prevents CRC mismatch at end)
 *  - Do not finalize inside DATA; switch to END state and finalize on CMD_END
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "etx_ota_update.h"
#include "main.h"
#include <stdarg.h>

static uint8_t Rx_Buffer[ ETX_OTA_PACKET_MAX_SIZE ];

static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;

static uint32_t ota_fw_total_size;
static uint32_t ota_fw_crc;
static uint32_t ota_fw_received_size;
static uint8_t  slot_num_to_write;

extern uint8_t updateComplete;
extern CRC_HandleTypeDef  hcrc;
extern UART_HandleTypeDef huart3;

ETX_GNRL_CFG_ *cfg_flash = (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR);

#ifndef ETX_TX_FN_T_DEFINED
typedef void (*etx_tx_fn_t)(const uint8_t *data, uint16_t len, void *ctx);
#define ETX_TX_FN_T_DEFINED 1
#endif
static etx_tx_fn_t s_resp_tx  = NULL;
static void       *s_resp_ctx = NULL;

static uint16_t      etx_receive_chunk(uint8_t *buf, uint16_t max_len);
static ETX_OTA_EX_   etx_process_data(uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num,
                                            uint8_t *data,
                                            uint16_t data_len,
                                            bool is_first_block);
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len);
static uint8_t       get_available_slot_number(void);
static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg);

static ETX_OTA_EX_   finalize_staged_image_and_mark(void);

static const char* ota_state_str(ETX_OTA_STATE_ s)
{
    switch (s) {
    case ETX_OTA_STATE_IDLE:   return "IDLE";
    case ETX_OTA_STATE_START:  return "START";
    case ETX_OTA_STATE_HEADER: return "HEADER";
    case ETX_OTA_STATE_DATA:   return "DATA";
    case ETX_OTA_STATE_END:    return "END";
    default: return "?";
    }
}

static uint32_t crc32_stm32_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= ((uint32_t)data[i] << 24);
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

/* -------------------- FORCED UART LOGGING -------------------- */
void etx_ota_send_text_raw(const char *s, uint16_t n)
{
    if (!s || n == 0) return;

    /* IMPORTANT: force all logs to UART so port 2000 stays clean for ACK/NACK */
    HAL_UART_Transmit(&huart3, (uint8_t*)s, n, HAL_MAX_DELAY);
}

void etx_ota_logf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof buf) n = sizeof(buf) - 1;
    etx_ota_send_text_raw(buf, (uint16_t)n);
}

static void ota_reset_session(void)
{
    OTA_LOGF("[OTA] reset_session()\r\n");
    ota_fw_total_size    = 0u;
    ota_fw_received_size = 0u;
    ota_fw_crc           = 0u;
    slot_num_to_write    = 0xFFu;
    ota_state            = ETX_OTA_STATE_START;
}

void etx_ota_set_resp_sender(etx_tx_fn_t fn, void *ctx)
{
    s_resp_tx  = fn;
    s_resp_ctx = ctx;
    OTA_LOGF("[OTA] resp_sender set: fn=%p ctx=%p\r\n", fn, ctx);
}

/* ACK/NACK response (this MUST remain TCP-capable) */
static void etx_ota_send_resp(uint8_t type)
{
    ETX_OTA_RESP_ rsp =
    {
        .sof         = ETX_OTA_SOF,
        .packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
        .data_len    = 1u,
        .status      = type,
        .eof         = ETX_OTA_EOF
    };

    rsp.crc = crc32_stm32_bytes(&rsp.status, 1u);

    if (s_resp_tx) {
        s_resp_tx((const uint8_t *)&rsp, (uint16_t)sizeof(rsp), s_resp_ctx);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t *)&rsp, sizeof(rsp), HAL_MAX_DELAY);
    }
}

ETX_OTA_EX_ etx_ota_feed_frame(const uint8_t *frame, uint16_t len)
{
    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

    do {
        if (!frame || len < (1 + 1 + 2 + 4 + 1)) {
            OTA_LOGF("[OTA] feed_frame: bad args frame=%p len=%u\r\n", frame, len);
            break;
        }

        if (frame[0] != ETX_OTA_SOF || frame[len-1] != ETX_OTA_EOF) {
            OTA_LOGF("[OTA] feed_frame: bad SOF/EOF (SOF=0x%02X EOF=0x%02X) len=%u\r\n",
                     frame[0], frame[len-1], len);
            break;
        }

        uint8_t  typ   = frame[1];
        uint16_t dlen  = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
        uint32_t need  = (uint32_t)(1 + 1 + 2) + dlen + 4 + 1;

        OTA_LOGF("[OTA] RX frame: type=%u dlen=%u total=%u state=%s\r\n",
                 (unsigned)typ, (unsigned)dlen, (unsigned)len, ota_state_str(ota_state));

        if (len != need) {
            OTA_LOGF("[OTA] feed_frame: length mismatch got=%u need=%lu\r\n",
                     (unsigned)len, (unsigned long)need);
            break;
        }

        if (len > ETX_OTA_PACKET_MAX_SIZE) {
            OTA_LOGF("[OTA] feed_frame: oversize packet len=%u max=%u\r\n",
                     (unsigned)len, (unsigned)ETX_OTA_PACKET_MAX_SIZE);
            etx_ota_send_resp(ETX_OTA_NACK);
            break;
        }

        const uint8_t* payload = &frame[4];
        uint32_t rec_crc =  (uint32_t)payload[dlen]
                          | ((uint32_t)payload[dlen + 1] << 8)
                          | ((uint32_t)payload[dlen + 2] << 16)
                          | ((uint32_t)payload[dlen + 3] << 24);
        uint32_t cal_crc = crc32_stm32_bytes(payload, dlen);

        if (typ == ETX_OTA_PACKET_TYPE_HEADER) {
            OTA_LOGF("[OTA] HDR bytes: %02X %02X %02X %02X %02X %02X %02X %02X ...\r\n",
                     frame[0], frame[1], frame[2], frame[3],
                     frame[4], frame[5], frame[6], frame[7]);
        }

        if (cal_crc != rec_crc) {
            OTA_LOGF("[OTA] CRC mismatch: calc=0x%08lX rec=0x%08lX typ=%u dlen=%u\r\n",
                     (unsigned long)cal_crc, (unsigned long)rec_crc,
                     (unsigned)typ, (unsigned)dlen);
            etx_ota_send_resp(ETX_OTA_NACK);
            break;
        }

        if (ota_state == ETX_OTA_STATE_IDLE) {
            if (typ == ETX_OTA_PACKET_TYPE_CMD && dlen == 1 && payload[0] == ETX_OTA_CMD_START) {
                OTA_LOGF("[OTA] IDLE->START by CMD_START\r\n");
                ota_reset_session();
            } else {
                OTA_LOGF("[OTA] In IDLE but got non-START -> NACK\r\n");
                etx_ota_send_resp(ETX_OTA_NACK);
                break;
            }
        }

        memcpy(Rx_Buffer, frame, len);
        ret = etx_process_data(Rx_Buffer, (uint16_t)len);

        OTA_LOGF("[OTA] process ret=%u -> RESP=%s\r\n",
                 (unsigned)ret, (ret == ETX_OTA_EX_OK) ? "ACK" : "NACK");

        etx_ota_send_resp((ret == ETX_OTA_EX_OK) ? ETX_OTA_ACK : ETX_OTA_NACK);
    } while (false);

    return ret;
}

/* UART blocking path (unchanged) */
ETX_OTA_EX_ etx_ota_download_and_flash(void)
{
    ETX_OTA_EX_ ret  = ETX_OTA_EX_OK;
    uint16_t    len;

    ota_fw_total_size    = 0u;
    ota_fw_received_size = 0u;
    ota_fw_crc           = 0u;
    ota_state            = ETX_OTA_STATE_START;
    slot_num_to_write    = 0xFFu;

    do {
        memset(Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE);
        len = etx_receive_chunk(Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE);

        if (len != 0u) {
            ret = etx_process_data(Rx_Buffer, len);
        } else {
            ret = ETX_OTA_EX_ERR;
        }

        if (ret != ETX_OTA_EX_OK) {
            OTA_LOGF("[OTA] UART path NACK\r\n");
            etx_ota_send_resp(ETX_OTA_NACK);
            break;
        } else {
            etx_ota_send_resp(ETX_OTA_ACK);
        }
    } while (ota_state != ETX_OTA_STATE_IDLE);

    return ret;
}

static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len)
{
    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

    do {
        if ((buf == NULL) || (len == 0u)) {
            OTA_LOGF("[OTA] process_data: bad args buf=%p len=%u\r\n", buf, len);
            break;
        }

        OTA_LOGF("[OTA] process_data: state=%s len=%u\r\n", ota_state_str(ota_state), (unsigned)len);

        ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*)buf;
        if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD && cmd->cmd == ETX_OTA_CMD_ABORT) {
            OTA_LOGF("[OTA] ABORT received\r\n");
            break;
        }

        switch (ota_state) {

        case ETX_OTA_STATE_IDLE:
            ret = ETX_OTA_EX_OK;
            break;

        case ETX_OTA_STATE_START:
        {
            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
            OTA_LOGF("[OTA] START: pkt_type=%u cmd=%u\r\n", (unsigned)c->packet_type, (unsigned)c->cmd);
            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_START) {
                OTA_LOGF("[OTA] Started...\r\n");
                ota_state = ETX_OTA_STATE_HEADER;
                ret = ETX_OTA_EX_OK;
            }
        } break;

        case ETX_OTA_STATE_HEADER:
        {
            ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*)buf;
            OTA_LOGF("[OTA] HEADER: pkt_type=%u dlen=%u\r\n",
                     (unsigned)header->packet_type, (unsigned)header->data_len);

            if (header->packet_type == ETX_OTA_PACKET_TYPE_HEADER) {
                ota_fw_total_size = header->meta_data.package_size;
                ota_fw_crc        = header->meta_data.package_crc;

                OTA_LOGF("[OTA] HEADER meta: size=%lu crc=0x%08lX\r\n",
                         (unsigned long)ota_fw_total_size,
                         (unsigned long)ota_fw_crc);

                ETX_GNRL_CFG_ cfg;
                memcpy(&cfg, cfg_flash, sizeof(cfg));
                OTA_LOGF("[OTA] CFG: s0 act=%u inv=%u run=%u | s1 act=%u inv=%u run=%u\r\n",
                         cfg.slot_table[0].is_this_slot_active,
                         cfg.slot_table[0].is_this_slot_not_valid,
                         cfg.slot_table[0].should_we_run_this_fw,
                         cfg.slot_table[1].is_this_slot_active,
                         cfg.slot_table[1].is_this_slot_not_valid,
                         cfg.slot_table[1].should_we_run_this_fw);

                slot_num_to_write = get_available_slot_number();
                OTA_LOGF("[OTA] slot_num_to_write=%u\r\n", (unsigned)slot_num_to_write);

                if (slot_num_to_write != 0xFFu) {
                    ota_state = ETX_OTA_STATE_DATA;
                    OTA_LOGF("[OTA] HEADER OK -> state=DATA\r\n");
                    ret = ETX_OTA_EX_OK;
                } else {
                    OTA_LOGF("[OTA] No available slot -> ERR\r\n");
                }
            }
        } break;

        case ETX_OTA_STATE_DATA:
        {
            /* IMPORTANT FIX: parse dlen from raw bytes to avoid struct packing issues */
            uint8_t  pkt_type = buf[1];
            uint16_t dlen     = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));

            OTA_LOGF("[OTA] DATA: pkt_type=%u dlen=%u rx_total=%lu/%lu\r\n",
                     (unsigned)pkt_type,
                     (unsigned)dlen,
                     (unsigned long)ota_fw_received_size,
                     (unsigned long)ota_fw_total_size);

            if (pkt_type == ETX_OTA_PACKET_TYPE_DATA) {

                /* FIX: reject extra packets beyond total size */
                if (ota_fw_received_size >= ota_fw_total_size) {
                    OTA_LOGF("[OTA] Extra DATA after completion -> NACK\r\n");
                    ret = ETX_OTA_EX_ERR;
                    break;
                }

                uint32_t remaining = ota_fw_total_size - ota_fw_received_size;
                if ((uint32_t)dlen > remaining) {
                    OTA_LOGF("[OTA] DATA overflow: dlen=%u remaining=%lu (rx=%lu total=%lu) -> NACK\r\n",
                             (unsigned)dlen,
                             (unsigned long)remaining,
                             (unsigned long)ota_fw_received_size,
                             (unsigned long)ota_fw_total_size);
                    ret = ETX_OTA_EX_ERR;
                    break;
                }

                bool first_block = (ota_fw_received_size == 0);

                if (first_block) {
                    OTA_LOGF("[OTA] First DATA block\r\n");
                    ETX_GNRL_CFG_ cfg;
                    memcpy(&cfg, cfg_flash, sizeof(cfg));
                    cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;
                    HAL_StatusTypeDef rcfg = write_cfg_to_flash(&cfg);
                    OTA_LOGF("[OTA] write_cfg_to_flash(first_block) rc=%d\r\n", (int)rcfg);
                    if (rcfg != HAL_OK) break;
                }

                HAL_StatusTypeDef ex = write_data_to_slot(slot_num_to_write, buf + 4, dlen, first_block);
                OTA_LOGF("[OTA] write_data_to_slot rc=%d new_rx=%lu\r\n",
                         (int)ex, (unsigned long)ota_fw_received_size);

                if (ex == HAL_OK) {
                    /* FIX: only complete when EXACTLY equal; then wait for CMD_END */
                    if (ota_fw_received_size == ota_fw_total_size) {
                        ota_state = ETX_OTA_STATE_END;
                        OTA_LOGF("[OTA] All bytes received -> state=END (wait CMD_END)\r\n");
                        ret = ETX_OTA_EX_OK;
                        break;
                    }

                    ret = ETX_OTA_EX_OK;
                } else {
                    OTA_LOGF("[OTA] DATA write error!\r\n");
                }
            }
        } break;

        case ETX_OTA_STATE_END:
        {
            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
            OTA_LOGF("[OTA] END: pkt_type=%u cmd=%u\r\n", (unsigned)c->packet_type, (unsigned)c->cmd);
            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_END) {
                OTA_LOGF("[OTA] END received -> finalize\r\n");
                ret = finalize_staged_image_and_mark();
                OTA_LOGF("[OTA] finalize ret=%u\r\n", (unsigned)ret);

                if (ret == ETX_OTA_EX_OK) {
                    ota_state = ETX_OTA_STATE_IDLE;
                    updateComplete = 1;
                    OTA_LOGF("[OTA] DONE -> state=IDLE updateComplete=1\r\n");
                }
            }
        } break;

        default:
            OTA_LOGF("[OTA] default: unknown state\r\n");
            ret = ETX_OTA_EX_ERR;
            break;
        }

    } while (false);

    return ret;
}

/* ----------- the rest of your functions are unchanged ----------- */

static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num,
                                            uint8_t *data,
                                            uint16_t data_len,
                                            bool is_first_block)
{
    HAL_StatusTypeDef ret;

    do {
        if (slot_num >= ETX_NO_OF_SLOTS) { ret = HAL_ERROR; break; }

        ret = HAL_FLASH_Unlock();
        if (ret != HAL_OK) break;

        if (is_first_block) {
            OTA_LOGF("[OTA] Erasing Slot %d...\r\n", slot_num);
            FLASH_EraseInitTypeDef EraseInitStruct;
            uint32_t SectorError;

            EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
            EraseInitStruct.Sector       = (slot_num == 0) ? FLASH_SECTOR_7 : FLASH_SECTOR_9;
            EraseInitStruct.NbSectors    = 2;
            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

            ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
            if (ret != HAL_OK) { OTA_LOGF("[OTA] Erase error\r\n"); break; }
        }

        uint32_t flash_addr = (slot_num == 0) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;

        for (uint32_t i = 0; i < (uint32_t)data_len; i++) {
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                                    (flash_addr + ota_fw_received_size),
                                    data[i]);
            if (ret == HAL_OK) {
                ota_fw_received_size += 1u;
            } else {
                OTA_LOGF("[OTA] Write error\r\n");
                break;
            }
        }

        if (ret != HAL_OK) break;

        ret = HAL_FLASH_Lock();
        if (ret != HAL_OK) break;

    } while (false);

    return ret;
}

/* ... keep the rest of your functions exactly as you already have them ... */

static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len)
{
    HAL_StatusTypeDef ret;

    do {
        ret = HAL_FLASH_Unlock();
        if (ret != HAL_OK) break;

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

        OTA_LOGF("[OTA] Erasing App region...\r\n");
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError;

        EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector       = FLASH_SECTOR_5;
        EraseInitStruct.NbSectors    = 2;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
        if (ret != HAL_OK) { OTA_LOGF("[OTA] App erase error\r\n"); break; }

        for (uint32_t i = 0; i < data_len; i++) {
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                                    (ETX_APP_FLASH_ADDR + i),
                                    data[i]);
            if (ret != HAL_OK) { OTA_LOGF("[OTA] App write error\r\n"); break; }
        }

        if (ret != HAL_OK) break;

        ret = HAL_FLASH_Lock();
        if (ret != HAL_OK) break;

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

    } while (false);

    return ret;
}

static ETX_OTA_EX_ finalize_staged_image_and_mark(void)
{
    uint32_t slot_addr = (slot_num_to_write == 0u) ? ETX_APP_SLOT0_FLASH_ADDR
                                                   : ETX_APP_SLOT1_FLASH_ADDR;

    OTA_LOGF("[OTA] finalize: slot=%u addr=0x%08lX size=%lu exp_crc=0x%08lX\r\n",
             (unsigned)slot_num_to_write,
             (unsigned long)slot_addr,
             (unsigned long)ota_fw_total_size,
             (unsigned long)ota_fw_crc);

    uint32_t cal_crc = crc32_stm32_bytes((const uint8_t*)slot_addr, ota_fw_total_size);
    if (cal_crc != ota_fw_crc) {
        OTA_LOGF("[OTA] CRC mismatch: calc=0x%08lX expected=0x%08lX\r\n",
                 (unsigned long)cal_crc, (unsigned long)ota_fw_crc);
        return ETX_OTA_EX_ERR;
    }
    OTA_LOGF("[OTA] CRC OK\r\n");

    ETX_GNRL_CFG_ cfg;
    memcpy(&cfg, cfg_flash, sizeof(cfg));

    cfg.slot_table[slot_num_to_write].fw_crc                 = cal_crc;
    cfg.slot_table[slot_num_to_write].fw_size                = ota_fw_total_size;
    cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 0u;
    cfg.slot_table[slot_num_to_write].should_we_run_this_fw  = 1u;

    for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
        if (i != slot_num_to_write) { cfg.slot_table[i].should_we_run_this_fw = 0u; }
    }

    cfg.reboot_cause = ETX_NORMAL_BOOT;

    HAL_StatusTypeDef wr = write_cfg_to_flash(&cfg);
    OTA_LOGF("[OTA] write_cfg_to_flash(finalize) rc=%d\r\n", (int)wr);
    if (wr != HAL_OK) {
        OTA_LOGF("[OTA] Config write error\r\n");
        return ETX_OTA_EX_ERR;
    }

    return ETX_OTA_EX_OK;
}

static uint8_t get_available_slot_number(void)
{
    uint8_t slot_number = 0xFF;

    ETX_GNRL_CFG_ cfg;
    memcpy(&cfg, cfg_flash, sizeof(cfg));

    for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
        if ((cfg.slot_table[i].is_this_slot_not_valid != 0u) ||
            (cfg.slot_table[i].is_this_slot_active == 0u)) {
            slot_number = i;
            OTA_LOGF("[OTA] Using Slot %u\r\n", slot_number);
            break;
        }
    }
    return slot_number;
}

void load_new_app(void)
{
    bool              is_update_available = false;
    uint8_t           slot_num = 0;
    HAL_StatusTypeDef ret;

    ETX_GNRL_CFG_ cfg;
    memcpy(&cfg, cfg_flash, sizeof(cfg));

    for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
        if (cfg.slot_table[i].should_we_run_this_fw == 1u) {
            OTA_LOGF("[OTA] New app staged in slot %u\r\n", i);
            is_update_available = true;
            slot_num = i;
            cfg.slot_table[i].is_this_slot_active   = 1u;
            cfg.slot_table[i].should_we_run_this_fw = 0u;
            break;
        }
    }

    if (is_update_available) {
        for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
            if (slot_num != i) cfg.slot_table[i].is_this_slot_active = 0u;
        }

        uint32_t slot_addr = (slot_num == 0u) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;

        ret = write_data_to_flash_app((uint8_t*)slot_addr, cfg.slot_table[slot_num].fw_size);
        if (ret != HAL_OK) {
            OTA_LOGF("[OTA] App flash write error\r\n");
        } else {
            ret = write_cfg_to_flash(&cfg);
            if (ret != HAL_OK) {
                OTA_LOGF("[OTA] Config write error\r\n");
            }
        }
    } else {
        for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
            if (cfg.slot_table[i].is_this_slot_active == 1u) { slot_num = i; break; }
        }
    }

    OTA_LOGF("[OTA] Verifying application...\r\n");
    uint32_t cal_data_crc = crc32_stm32_bytes((const uint8_t*)ETX_APP_FLASH_ADDR,
                                             cfg.slot_table[slot_num].fw_size);

    if (cal_data_crc != cfg.slot_table[slot_num].fw_crc) {
        OTA_LOGF("[OTA] Invalid application CRC.\r\n");
    } else {
        OTA_LOGF("[OTA] Application verified.\r\n");
    }
}

static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg)
{
    HAL_StatusTypeDef ret;

    do {
        if (cfg == NULL) { ret = HAL_ERROR; break; }

        ret = HAL_FLASH_Unlock();
        if (ret != HAL_OK) break;

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError;

        EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector       = FLASH_SECTOR_4;
        EraseInitStruct.NbSectors    = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

        ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
        if (ret != HAL_OK) break;

        uint8_t *data = (uint8_t *)cfg;
        for (uint32_t i = 0u; i < sizeof(ETX_GNRL_CFG_); i++) {
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, ETX_CONFIG_FLASH_ADDR + i, data[i]);
            if (ret != HAL_OK) { OTA_LOGF("[OTA] Config write error\r\n"); break; }
        }

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);
        if (ret != HAL_OK) break;

        ret = HAL_FLASH_Lock();
        if (ret != HAL_OK) break;

    } while (false);

    return ret;
}
