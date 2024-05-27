/*
 * ov7670.c
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */
#include <stdio.h>
#include "main.h"
#include "ov7670.h"
#include "ov7670Reg.h"


/*** Internal Static Variables ***/
static DCMI_HandleTypeDef *sp_hdcmi;
static I2C_HandleTypeDef  *sp_hi2c;

static uint32_t    s_destAddressForContiuousMode;
static uint32_t    s_currentH;
static uint32_t    s_currentV;
static void 	   (* s_cbHsync)(uint32_t h);
static void        (* s_cbVsync)(uint32_t v);

uint32_t frameCounter = 0;

/*** Internal Function Declarations ***/
static HAL_StatusTypeDef OV7670_Write(uint8_t regAddr, uint8_t data);
static HAL_StatusTypeDef OV7670_Read(uint8_t regAddr, uint8_t *data);

/*** External Function Defines ***/
uint8_t OV7670_Init(DCMI_HandleTypeDef *p_hdcmi, I2C_HandleTypeDef *p_hi2c)
{
	sp_hdcmi     = p_hdcmi;
	sp_hi2c      = p_hi2c;
	s_destAddressForContiuousMode = 0;

	HAL_StatusTypeDef status;
	uint8_t errNum = 0;

	OV7670_ResetHW();

	status = OV7670_ResetSW();
	errNum += ( status != HAL_OK );
	HAL_Delay(30);

	uint8_t buffer[4];
	status = OV7670_Read(0x0b, buffer);
	errNum += ( status != HAL_OK );

	if(buffer[0] != 0x73)
		return 255;

	return errNum;
}

uint8_t OV7670_UploadSettings()
{
	HAL_StatusTypeDef status;
	uint8_t errNum = 0;

	status = OV7670_StopCap();
	errNum += ( status != HAL_OK );

	status = OV7670_ResetSW();
	errNum += ( status != HAL_OK );
	HAL_Delay(30);

	for(int i = 0; OV7670_reg[i][0] != REG_END; i++)
	{
		status = OV7670_Write(OV7670_reg[i][0], OV7670_reg[i][1]);
		errNum += ( status != HAL_OK );
		HAL_Delay(1);
	}

  return errNum;
}

HAL_StatusTypeDef OV7670_StartCap(uint32_t capMode, uint32_t destAddress)
{
	HAL_StatusTypeDef ret;

	ret = OV7670_StopCap();
	if (capMode == OV7670_CAP_CONTINUOUS)
	{
		s_destAddressForContiuousMode = destAddress;
		ret |= HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress, OV7670_CAP_WIDTH * OV7670_CAP_HEIGHT/2);
	}
	else if (capMode == OV7670_CAP_SINGLE_FRAME)
	{
		s_destAddressForContiuousMode = 0;
		ret |= HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_CAP_WIDTH * OV7670_CAP_HEIGHT/2);
	}

  return ret;
}

HAL_StatusTypeDef OV7670_StopCap()
{
	return HAL_DCMI_Stop(sp_hdcmi);
}

HAL_StatusTypeDef OV7670_ResetSW()
{
	return OV7670_Write(0x12, 0x80);
}

void OV7670_ResetHW()
{
	HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void OV7670_RegisterCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v))
{
	s_cbHsync = cbHsync;
	s_cbVsync = cbVsync;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	printf("FRAME %lu\n", HAL_GetTick());
	frameCounter++;

	if(s_cbVsync)
		s_cbVsync(s_currentV);

	if(s_destAddressForContiuousMode != 0)
	{
    HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_CAP_WIDTH * OV7670_CAP_HEIGHT/2);
	}

	s_currentV++;
	s_currentH = 0;
}



/*** Internal Function Defines ***/

static HAL_StatusTypeDef OV7670_Write(uint8_t regAddr, uint8_t data)
{
	HAL_StatusTypeDef ret;
	do {
    ret = HAL_I2C_Mem_Write(sp_hi2c, OV7670_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	} while (ret != HAL_OK && 0);

	return ret;
}

static HAL_StatusTypeDef OV7670_Read(uint8_t regAddr, uint8_t *data)
{
	HAL_StatusTypeDef ret;
	do {
    ret = HAL_I2C_Master_Transmit(sp_hi2c, OV7670_ADDR, &regAddr, 1, 100);
    ret |= HAL_I2C_Master_Receive(sp_hi2c, OV7670_ADDR, data, 1, 100);
	} while (ret != HAL_OK && 0);

	return ret;
}


