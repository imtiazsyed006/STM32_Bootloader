/*
 * bl_flag.c
 *
 *  Created on: Jan 20, 2026
 *      Author: Imtiaz
 */


#include "bl_flag.h"

void BL_Flag_InitBackupDomain(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();            // allow write to backup domain :contentReference[oaicite:3]{index=3}
    // Optional but commonly done:
    __HAL_RCC_RTC_ENABLE();                // if RTC clocking is used in your project
}

void BL_Flag_SetOtaRequest(RTC_HandleTypeDef *hrtc)
{
    BL_Flag_InitBackupDomain();
    HAL_RTCEx_BKUPWrite(hrtc, BL_OTA_BKP_REG, BL_OTA_MAGIC);
}

uint8_t BL_Flag_IsOtaRequested(RTC_HandleTypeDef *hrtc)
{
    uint32_t v = HAL_RTCEx_BKUPRead(hrtc, BL_OTA_BKP_REG);
    return (v == BL_OTA_MAGIC) ? 1u : 0u;
}

void BL_Flag_ClearOtaRequest(RTC_HandleTypeDef *hrtc)
{
    BL_Flag_InitBackupDomain();
    HAL_RTCEx_BKUPWrite(hrtc, BL_OTA_BKP_REG, 0u);
}
