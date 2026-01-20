/*
 * etx_ota_update.c (BOOTLOADER + RTOS SAFE)  [CMSIS-OS v1]
 *
 * - Port 2001 RX bytes -> comm_ota_rx_bytes() assembles frames -> etx_ota_post_frame()
 * - This file runs the OTA state machine + FLASH programming in an RTOS worker task
 * - Port 2000 TX (ACK/NACK) is done via s_resp_tx callback (lwIP-safe wrapper using tcpip_callback)
 *
 * REQUIREMENT:
 *  - Call etx_ota_worker_start() once AFTER the scheduler starts (e.g. StartDefaultTask()).
 *
 * NOTES:
 *  - This version uses CMSIS-OS v1 APIs: osThreadDef/osThreadCreate, osMessageQDef/osMessageCreate,
 *    osMessagePut/osMessageGet, osMutexDef/osMutexCreate, etc.
 *  - No CMSIS-RTOS2 APIs are used.
 */

#include "etx_ota_update.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* RTOS (CMSIS-OS v1) */
#include "cmsis_os.h"

/* -------------------- Protocol constants -------------------- */
#ifndef ETX_OTA_SOF
#define ETX_OTA_SOF  0xAA
#endif
#ifndef ETX_OTA_EOF
#define ETX_OTA_EOF  0xBB
#endif

#ifndef ETX_OTA_PACKET_TYPE_RESPONSE
#define ETX_OTA_PACKET_TYPE_RESPONSE 3
#endif

#ifndef ETX_OTA_ACK
#define ETX_OTA_ACK  0x00
#endif
#ifndef ETX_OTA_NACK
#define ETX_OTA_NACK 0x01
#endif

/* -------------------- Your existing globals/externs -------------------- */
extern uint8_t updateComplete;
extern UART_HandleTypeDef huart3;

/* Config location as in your original code */
ETX_GNRL_CFG_ *cfg_flash = (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR);

/* -------------------- Response sender hook -------------------- */
/* tcpserverRAW.c sets this using: etx_ota_set_resp_sender(ota_resp_tx_lwip, g_port2000_pcb) */
#ifndef ETX_TX_FN_T_DEFINED
typedef void (*etx_tx_fn_t)(const uint8_t *data, uint16_t len, void *ctx);
#define ETX_TX_FN_T_DEFINED 1
#endif

static etx_tx_fn_t s_resp_tx  = NULL;
static void       *s_resp_ctx = NULL;

void etx_ota_set_resp_sender(etx_tx_fn_t fn, void *ctx)
{
    s_resp_tx  = fn;
    s_resp_ctx = ctx;
}

/* -------------------- CRC32 (STM32 poly) -------------------- */
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

/* -------------------- UART logging (optional) -------------------- */
static void ota_uart_write(const char *s)
{
    if (!s) return;
    HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void OTA_LOGF(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    buf[n] = 0;
    ota_uart_write(buf);
}

/* -------------------- Send ACK/NACK response frame -------------------- */
static void etx_ota_send_resp(uint8_t status)
{
    ETX_OTA_RESP_ rsp =
    {
        .sof         = ETX_OTA_SOF,
        .packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
        .data_len    = 1u,
        .status      = status,
        .eof         = ETX_OTA_EOF
    };

    rsp.crc = crc32_stm32_bytes(&rsp.status, 1u);

    if (s_resp_tx) {
        /* Must be lwIP-safe wrapper (tcpip_callback) */
        s_resp_tx((const uint8_t*)&rsp, (uint16_t)sizeof(rsp), s_resp_ctx);
    } else {
        /* fallback: UART */
        HAL_UART_Transmit(&huart3, (uint8_t*)&rsp, (uint16_t)sizeof(rsp), HAL_MAX_DELAY);
    }
}

/* -------------------- OTA state machine -------------------- */
static uint8_t Rx_Buffer[ETX_OTA_PACKET_MAX_SIZE];

static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;
static uint32_t ota_fw_total_size;
static uint32_t ota_fw_crc;
static uint32_t ota_fw_received_size;
static uint8_t  slot_num_to_write;

/* forward decls (as in your original file) */
static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num, uint8_t *data, uint16_t data_len, bool is_first_block);
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len);
static uint8_t get_available_slot_number(void);
static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg);
static ETX_OTA_EX_ finalize_staged_image_and_mark(void);

/* If you have this in your project, great. If not, comment logs that use it. */
#ifndef OTA_HAVE_STATE_STR
/* #define OTA_HAVE_STATE_STR 1 */
#endif
#ifdef OTA_HAVE_STATE_STR
extern const char* ota_state_str(ETX_OTA_STATE_ s);
#endif

static void ota_reset_session(void)
{
    ota_fw_total_size    = 0u;
    ota_fw_received_size = 0u;
    ota_fw_crc           = 0u;
    slot_num_to_write    = 0xFFu;
    ota_state            = ETX_OTA_STATE_START;
}
void goto_application(void)
{
  printf("Jumping to application...\r\n");

  void (*app_reset_handler)(void) =
      (void*)(*((volatile uint32_t*) (ETX_APP_FLASH_ADDR + 4U)));

  // Turn OFF LED before leaving BL
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_RCC_DeInit();
  HAL_DeInit();
  __set_MSP(*(volatile uint32_t*) ETX_APP_FLASH_ADDR);
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  app_reset_handler();
}
/* =======================================================================
 * CMSIS-OS v1 Worker + Queue (Frame processing outside tcpip_thread)
 * ======================================================================= */

typedef struct {
    uint16_t len;
    uint8_t  buf[ETX_OTA_PACKET_MAX_SIZE];
} ota_frame_t;

#define OTA_POOL_ITEMS   8u

static ota_frame_t  s_pool_mem[OTA_POOL_ITEMS];
static uint8_t      s_pool_used[OTA_POOL_ITEMS];

static osMutexId    s_pool_mutex = NULL;
static osMessageQId s_q          = NULL;
static osThreadId   s_tid        = NULL;

/* worker prototype must match CMSIS-OS v1 */
static void ota_worker(void const *arg);

/* CMSIS-OS v1 object definitions */
osMutexDef(OTA_POOL_MUTEX);
osMessageQDef(OTA_Q, OTA_POOL_ITEMS, uint32_t);
osThreadDef(OTA_WORKER, ota_worker, osPriorityNormal, 0, 2048);

static ota_frame_t* pool_alloc(void)
{
    ota_frame_t *f = NULL;
    if (!s_pool_mutex) return NULL;

    osMutexWait(s_pool_mutex, osWaitForever);
    for (uint32_t i = 0; i < OTA_POOL_ITEMS; i++) {
        if (s_pool_used[i] == 0u) {
            s_pool_used[i] = 1u;
            f = &s_pool_mem[i];
            break;
        }
    }
    osMutexRelease(s_pool_mutex);

    return f;
}

static void pool_free(ota_frame_t *f)
{
    if (!f || !s_pool_mutex) return;

    osMutexWait(s_pool_mutex, osWaitForever);
    for (uint32_t i = 0; i < OTA_POOL_ITEMS; i++) {
        if (f == &s_pool_mem[i]) {
            s_pool_used[i] = 0u;
            break;
        }
    }
    osMutexRelease(s_pool_mutex);
}

/* Call once after kernel is running */
void etx_ota_worker_start(void)
{
    if (s_tid) return;

    memset(s_pool_used, 0, sizeof(s_pool_used));

    s_pool_mutex = osMutexCreate(osMutex(OTA_POOL_MUTEX));
    s_q          = osMessageCreate(osMessageQ(OTA_Q), NULL);

    if (!s_pool_mutex || !s_q) {
        OTA_LOGF("[OTA] worker_start FAILED mutex=%p q=%p\r\n", s_pool_mutex, s_q);
        return;
    }

    s_tid = osThreadCreate(osThread(OTA_WORKER), NULL);
    OTA_LOGF("[OTA] worker_start tid=%p\r\n", s_tid);
}

/* comm.c posts complete frames here (fast + non-blocking) */
void etx_ota_post_frame(const uint8_t *frame, uint16_t len)
{
    if (!frame || len == 0) return;
    if (!s_q || !s_pool_mutex) return;
    if (len > ETX_OTA_PACKET_MAX_SIZE) return;

    ota_frame_t *f = pool_alloc();
    if (!f) {
        /* pool exhausted: drop frame */
        return;
    }

    f->len = len;
    memcpy(f->buf, frame, len);

    /* queue pointer as uint32_t (OK on STM32F7 - 32-bit pointers) */
    osStatus st = osMessagePut(s_q, (uint32_t)f, 0);
    if (st != osOK) {
        pool_free(f);
    }
}

/* Worker processes: validate envelope+CRC -> etx_process_data() -> respond */
static void ota_worker(void const *arg)
{
    (void)arg;

    for (;;) {
        osEvent evt = osMessageGet(s_q, osWaitForever);
        if (evt.status != osEventMessage) {
            continue;
        }

        ota_frame_t *f = (ota_frame_t*)evt.value.p;
        if (!f) continue;

        ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

        do {
            if (f->len < (1u+1u+2u+4u+1u)) break;
            if (f->buf[0] != ETX_OTA_SOF) break;
            if (f->buf[f->len - 1] != ETX_OTA_EOF) break;

            uint8_t  typ  = f->buf[1];
            uint16_t dlen = (uint16_t)(f->buf[2] | ((uint16_t)f->buf[3] << 8));

            uint32_t need = (uint32_t)(1u+1u+2u) + (uint32_t)dlen + 4u + 1u;
            if (need != f->len) break;

            const uint8_t *payload = &f->buf[4];

            uint32_t rec_crc =  (uint32_t)payload[dlen]
                              | ((uint32_t)payload[dlen + 1] << 8)
                              | ((uint32_t)payload[dlen + 2] << 16)
                              | ((uint32_t)payload[dlen + 3] << 24);

            uint32_t cal_crc = crc32_stm32_bytes(payload, dlen);
            if (cal_crc != rec_crc) {
                OTA_LOGF("[OTA] CRC mismatch typ=%u dlen=%u calc=0x%08lX rec=0x%08lX\r\n",
                         (unsigned)typ, (unsigned)dlen,
                         (unsigned long)cal_crc, (unsigned long)rec_crc);
                ret = ETX_OTA_EX_ERR;
                break;
            }

            /* Gate IDLE: must start with CMD_START */
            if (ota_state == ETX_OTA_STATE_IDLE) {
                if (typ == ETX_OTA_PACKET_TYPE_CMD && dlen == 1 && payload[0] == ETX_OTA_CMD_START) {
                    ota_reset_session();
                } else {
                    ret = ETX_OTA_EX_ERR;
                    break;
                }
            }

            memcpy(Rx_Buffer, f->buf, f->len);
            ret = etx_process_data(Rx_Buffer, f->len);

        } while (0);

        etx_ota_send_resp((ret == ETX_OTA_EX_OK) ? ETX_OTA_ACK : ETX_OTA_NACK);
        pool_free(f);
    }
}

/* =======================================================================
 * Your OTA logic below (mostly unchanged from what you pasted)
 * ======================================================================= */

/* If you still keep the old UART receive path, ensure etx_receive_chunk exists.
 * If not, you can delete etx_ota_download_and_flash() to avoid link errors.
 */
#ifdef ETX_OTA_ENABLE_UART_PATH
extern uint16_t etx_receive_chunk(uint8_t *buf, uint16_t max_len);

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
#endif /* ETX_OTA_ENABLE_UART_PATH */

static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len)
{
    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

    do {
        if ((buf == NULL) || (len == 0u)) {
            OTA_LOGF("[OTA] process_data: bad args buf=%p len=%u\r\n", buf, len);
            break;
        }

#ifdef OTA_HAVE_STATE_STR
        OTA_LOGF("[OTA] process_data: state=%s len=%u\r\n", ota_state_str(ota_state), (unsigned)len);
#else
        OTA_LOGF("[OTA] process_data: state=%u len=%u\r\n", (unsigned)ota_state, (unsigned)len);
#endif

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
            /* IMPORTANT: parse dlen from raw bytes to avoid struct packing issues */
            uint8_t  pkt_type = buf[1];
            uint16_t dlen     = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));

            OTA_LOGF("[OTA] DATA: pkt_type=%u dlen=%u rx_total=%lu/%lu\r\n",
                     (unsigned)pkt_type,
                     (unsigned)dlen,
                     (unsigned long)ota_fw_received_size,
                     (unsigned long)ota_fw_total_size);

            if (pkt_type == ETX_OTA_PACKET_TYPE_DATA) {

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

                bool first_block = (ota_fw_received_size == 0u);

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

                  bl_send_text_2000_from_task("OTA_DONE\r\n");
                  bl_send_text_2000_from_task("BL_JUMPING\r\n");

                  osDelay(200);   // give lwIP time to push out packets

                  goto_application();
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
