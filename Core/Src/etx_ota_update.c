/*
 * etx_ota_update.c
 * UART3 + TCP-capable OTA flow: receive -> write slot -> mark -> load_new_app()
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

void etx_ota_send_text_raw(const char *s, uint16_t n)
{
    if (!s || n == 0) return;

    if (s_resp_tx) {
        s_resp_tx((const uint8_t*)s, n, s_resp_ctx);
    } else {

        HAL_UART_Transmit(&huart3, (uint8_t*)s, n, HAL_MAX_DELAY);
    }
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
}


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
        s_resp_tx((const uint8_t *)&rsp, sizeof(rsp), s_resp_ctx);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t *)&rsp, sizeof(rsp), HAL_MAX_DELAY);
    }
}

ETX_OTA_EX_ etx_ota_feed_frame(const uint8_t *frame, uint16_t len)
{
    ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

    do {
        if (!frame || len < (1 + 1 + 2 + 4 + 1)) break;
        if (frame[0] != ETX_OTA_SOF || frame[len-1] != ETX_OTA_EOF) break;

        uint8_t  typ   = frame[1];
        uint16_t dlen  = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
        uint32_t need  = (uint32_t)(1 + 1 + 2) + dlen + 4 + 1;
        if (len != need) break;

        const uint8_t* payload = &frame[4];

        uint32_t rec_crc =  (uint32_t)payload[dlen]
                          | ((uint32_t)payload[dlen + 1] << 8)
                          | ((uint32_t)payload[dlen + 2] << 16)
                          | ((uint32_t)payload[dlen + 3] << 24);
        uint32_t cal_crc = crc32_stm32_bytes(payload, dlen);

        if (cal_crc != rec_crc) {
        	OTA_LOGF("Chunk CRC mismatch calc=0x%08lX rec=0x%08lX\r\n",
                   (unsigned long)cal_crc, (unsigned long)rec_crc);
            etx_ota_send_resp(ETX_OTA_NACK);
            break;
        }

        if (ota_state == ETX_OTA_STATE_IDLE) {
            if (typ == ETX_OTA_PACKET_TYPE_CMD && dlen == 1 && payload[0] == ETX_OTA_CMD_START) {
                ota_reset_session();
            } else {
                etx_ota_send_resp(ETX_OTA_NACK);
                break;
            }
        }

        if (len > ETX_OTA_PACKET_MAX_SIZE) {
        	OTA_LOGF("Oversize packet\r\n");
            etx_ota_send_resp(ETX_OTA_NACK);
            break;
        }

        memcpy(Rx_Buffer, frame, len);
        ret = etx_process_data(Rx_Buffer, (uint16_t)len);
        etx_ota_send_resp((ret == ETX_OTA_EX_OK) ? ETX_OTA_ACK : ETX_OTA_NACK);
    } while (false);

    return ret;
}


ETX_OTA_EX_ etx_ota_download_and_flash(void)
{
    ETX_OTA_EX_ ret  = ETX_OTA_EX_OK;
    uint16_t    len;
    /* Reset state */
    ota_fw_total_size    = 0u;
    ota_fw_received_size = 0u;
    ota_fw_crc           = 0u;
    ota_state            = ETX_OTA_STATE_START;
    slot_num_to_write    = 0xFFu;

    do
    {
        memset(Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE);
        len = etx_receive_chunk(Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE);

        if (len != 0u) {
            ret = etx_process_data(Rx_Buffer, len);
        } else {
            ret = ETX_OTA_EX_ERR;
        }

        if (ret != ETX_OTA_EX_OK) {
        	OTA_LOGF("NACK\r\n");
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
        if ((buf == NULL) || (len == 0u)) break;

        ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*)buf;
        if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD && cmd->cmd == ETX_OTA_CMD_ABORT) {
            OTA_LOGF("OTA ABORT received\r\n");
            break;
        }

        switch (ota_state) {
        case ETX_OTA_STATE_IDLE:
            ret = ETX_OTA_EX_OK;
            break;

        case ETX_OTA_STATE_START:
        {
            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_START) {
            	OTA_LOGF("Started...\r\n");
                ota_state = ETX_OTA_STATE_HEADER;
                ret = ETX_OTA_EX_OK;
            }
        } break;

        case ETX_OTA_STATE_HEADER:
        {
            ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*)buf;
            if (header->packet_type == ETX_OTA_PACKET_TYPE_HEADER) {
                ota_fw_total_size = header->meta_data.package_size;
                ota_fw_crc        = header->meta_data.package_crc;
                OTA_LOGF("HEADER: size=%lu bytes\r\n", (unsigned long)ota_fw_total_size);
                slot_num_to_write = get_available_slot_number();
                if (slot_num_to_write != 0xFFu) {
                    ota_state = ETX_OTA_STATE_DATA;
                    ret = ETX_OTA_EX_OK;
                }
            }
        } break;

        case ETX_OTA_STATE_DATA:
        {
            ETX_OTA_DATA_ *data   = (ETX_OTA_DATA_*)buf;
            uint16_t       dlen   = data->data_len;

            if (data->packet_type == ETX_OTA_PACKET_TYPE_DATA) {
                HAL_StatusTypeDef ex;
                bool first_block = (ota_fw_received_size == 0);

                if (first_block) {
                    ETX_GNRL_CFG_ cfg;
                    memcpy(&cfg, cfg_flash, sizeof(cfg));
                    cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;
                    ret = write_cfg_to_flash(&cfg);
                    if (ret != ETX_OTA_EX_OK) break;
                }


                ex = write_data_to_slot(slot_num_to_write, buf + 4, dlen, first_block);
                if (ex == HAL_OK) {
                	OTA_LOGF("[%lu/%lu]\r\n",
                           (unsigned long)(ota_fw_received_size/ETX_OTA_DATA_MAX_SIZE),
                           (unsigned long)(ota_fw_total_size/ETX_OTA_DATA_MAX_SIZE));


                    if (ota_fw_received_size >= ota_fw_total_size) {
                        ret = finalize_staged_image_and_mark();
                        if (ret == ETX_OTA_EX_OK) {
                            ota_state = ETX_OTA_STATE_IDLE;
                            updateComplete = 1;
                        }
                        break;
                    }

                    ret = ETX_OTA_EX_OK;
                }
            }
        } break;

        case ETX_OTA_STATE_END:
        {

            ETX_OTA_COMMAND_ *c = (ETX_OTA_COMMAND_*)buf;
            if (c->packet_type == ETX_OTA_PACKET_TYPE_CMD && c->cmd == ETX_OTA_CMD_END) {
            	OTA_LOGF("END\r\nValidating staged image...\r\n");
                ret = finalize_staged_image_and_mark();
                if (ret == ETX_OTA_EX_OK) {
                    ota_state = ETX_OTA_STATE_IDLE;
                    updateComplete = 1;
                }
            }
        } break;

        default:
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
        	OTA_LOGF("Erasing Slot %d...\r\n", slot_num);
            FLASH_EraseInitTypeDef EraseInitStruct;
            uint32_t SectorError;

            EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
            EraseInitStruct.Sector       = (slot_num == 0) ? FLASH_SECTOR_7 : FLASH_SECTOR_9;
            EraseInitStruct.NbSectors    = 2;
            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

            ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
            if (ret != HAL_OK) { OTA_LOGF("Erase error\r\n"); break; }
        }

        uint32_t flash_addr = (slot_num == 0) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;

        for (uint32_t i = 0; i < (uint32_t)data_len; i++) {
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                                    (flash_addr + ota_fw_received_size),
                                    data[i]);
            if (ret == HAL_OK) {
                ota_fw_received_size += 1u;
            } else {
            	OTA_LOGF("Write error\r\n");
                break;
            }
        }

        if (ret != HAL_OK) break;

        ret = HAL_FLASH_Lock();
        if (ret != HAL_OK) break;

    } while (false);

    return ret;
}


static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len)
{
    HAL_StatusTypeDef ret;

    do {
        ret = HAL_FLASH_Unlock();
        if (ret != HAL_OK) break;

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

        OTA_LOGF("Erasing App region...\r\n");
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError;

        EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector       = FLASH_SECTOR_5;
        EraseInitStruct.NbSectors    = 2; // Sectors 5,6
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
        if (ret != HAL_OK) { OTA_LOGF("App erase error\r\n"); break; }

        for (uint32_t i = 0; i < data_len; i++) {
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                                    (ETX_APP_FLASH_ADDR + i),
                                    data[i]);
            if (ret != HAL_OK) { OTA_LOGF("App write error\r\n"); break; }
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

    uint32_t cal_crc = crc32_stm32_bytes((const uint8_t*)slot_addr, ota_fw_total_size);
    if (cal_crc != ota_fw_crc) {
    	OTA_LOGF("CRC mismatch: calc=0x%08lX expected=0x%08lX\r\n",
               (unsigned long)cal_crc, (unsigned long)ota_fw_crc);
        return ETX_OTA_EX_ERR;
    }
    OTA_LOGF("CRC OK\r\n");

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

    ETX_OTA_EX_ wr = write_cfg_to_flash(&cfg);
    if (wr != ETX_OTA_EX_OK) {
    	OTA_LOGF("Config write error\r\n");
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
            OTA_LOGF("Using Slot %u\r\n", slot_number);
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

        	OTA_LOGF("New app staged in slot %u\r\n", i);
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
        	OTA_LOGF("App flash write error\r\n");
        } else {
            ret = write_cfg_to_flash(&cfg);
            if (ret != HAL_OK) {
            	OTA_LOGF("Config write error\r\n");
            }
        }
    } else {

        for (uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++) {
            if (cfg.slot_table[i].is_this_slot_active == 1u) { slot_num = i; break; }
        }
    }


    OTA_LOGF("Verifying application...\r\n");


    uint32_t cal_data_crc = crc32_stm32_bytes((const uint8_t*)ETX_APP_FLASH_ADDR,
                                              cfg.slot_table[slot_num].fw_size);

    if (cal_data_crc != cfg.slot_table[slot_num].fw_crc) {
    	OTA_LOGF("Invalid application CRC.\r\n");
    } else {
    	OTA_LOGF("Application verified.\r\n");
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
            if (ret != HAL_OK) { OTA_LOGF("Config write error\r\n"); break; }
        }

        FLASH_WaitForLastOperation(HAL_MAX_DELAY);
        if (ret != HAL_OK) break;

        ret = HAL_FLASH_Lock();
        if (ret != HAL_OK) break;

    } while (false);

    return ret;
}
