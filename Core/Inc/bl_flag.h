/*
 * bl_flag.h
 *
 *  Created on: Jan 20, 2026
 *      Author: Imtiaz
 */

#ifndef BL_FLAG_H_
#define BL_FLAG_H_

#pragma once
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include "rtc.h"
#define BL_OTA_MAGIC   (0xB00710ADu)  // any 32-bit magic you like
#define BL_OTA_BKP_REG RTC_BKP_DR0    // choose DR0..DRx (don’t clash with RTC usage)

void     BL_Flag_InitBackupDomain(void);
void     BL_Flag_SetOtaRequest(RTC_HandleTypeDef *hrtc);
uint8_t  BL_Flag_IsOtaRequested(RTC_HandleTypeDef *hrtc);
void     BL_Flag_ClearOtaRequest(RTC_HandleTypeDef *hrtc);


#endif /* BL_FLAG_H_ */
