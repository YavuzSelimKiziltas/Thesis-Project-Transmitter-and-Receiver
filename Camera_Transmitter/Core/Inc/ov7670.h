/*
 * ov7670.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef OV7670_OV7670_H_
#define OV7670_OV7670_H_

#include "stm32f4xx_hal.h"

#define OV7670_CAP_CONTINUOUS   0
#define OV7670_CAP_SINGLE_FRAME 1

/*** Internal Const Values, Macros ***/
#define OV7670_CAP_WIDTH  320
#define OV7670_CAP_HEIGHT 200

uint8_t			  OV7670_Init(DCMI_HandleTypeDef *p_hdcmi, I2C_HandleTypeDef *p_hi2c);
uint8_t 		  OV7670_UploadSettings();
HAL_StatusTypeDef OV7670_StartCap(uint32_t capMode, uint32_t destAddress);
HAL_StatusTypeDef OV7670_StopCap();
HAL_StatusTypeDef OV7670_ResetSW();
void			  OV7670_ResetHW();
void 			  OV7670_RegisterCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));

#endif /* OV7670_OV7670_H_ */
