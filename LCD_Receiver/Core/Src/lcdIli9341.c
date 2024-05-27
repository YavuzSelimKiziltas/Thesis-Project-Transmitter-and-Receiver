/*
 * lcdIli9341.c
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lcdIli9341.h"

/*** Internal Const Values, Macros ***/
#define FSMC_ADDRESS  (0x60000000 + ((FSMC_NEx-1) << 26))
#define LCD_CMD_ADDR  (FSMC_ADDRESS)


#define LCD_DATA_ADDR (FSMC_ADDRESS | 1 << (FSMC_Ax + 1))
#define LCD_CMD       (*((volatile uint16_t*) LCD_CMD_ADDR))
#define LCD_DATA      (*((volatile uint16_t*) LCD_DATA_ADDR))



/*** Internal Function Declarations ***/
static void Ili9341_ReadData();

/*** External Function Defines ***/
void Ili9341_SetArea(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
  Ili9341_WriteCmd(0x2a);
  Ili9341_WriteData(xStart >> 8);
  Ili9341_WriteData(xStart & 0xff);
  Ili9341_WriteData(xEnd >> 8);
  Ili9341_WriteData(xEnd & 0xff);

  Ili9341_WriteCmd(0x2b);
  Ili9341_WriteData(yStart >> 8);
  Ili9341_WriteData(yStart & 0xff);
  Ili9341_WriteData(yEnd >> 8);
  Ili9341_WriteData(yEnd & 0xff);

  Ili9341_WriteCmd(0x2c);
}

void Ili9341_SetAreaRead(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
  Ili9341_WriteCmd(0x2a);
  Ili9341_WriteData(xStart >> 8);
  Ili9341_WriteData(xStart & 0xff);
  Ili9341_WriteData(xEnd >> 8);
  Ili9341_WriteData(xEnd & 0xff);

  Ili9341_WriteCmd(0x2b);
  Ili9341_WriteData(yStart >> 8);
  Ili9341_WriteData(yStart & 0xff);
  Ili9341_WriteData(yEnd >> 8);
  Ili9341_WriteData(yEnd & 0xff);

  Ili9341_WriteCmd(0x2e);

  // the first read is invalid
  Ili9341_ReadData();
}

void Ili9341_DrawRect(uint16_t xStart, uint16_t yStart, uint16_t width, uint16_t height, uint16_t color)
{
  Ili9341_SetArea(xStart, yStart, xStart + width - 1, yStart + height - 1);
  for( uint16_t y = 0; y < height; y++ ){
    for( uint16_t x = 0; x < width; x++ ){
      LCD_DATA = color;
    }
  }
}

inline uint16_t* Ili9341_GetDrawAddress()
{
  return (uint16_t*)LCD_DATA_ADDR;
}

void Ili9341_Init(uint16_t width, uint16_t height)
{
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);HAL_Delay(10);

	Ili9341_WriteCmd(0x01); 		//software reset
	HAL_Delay(50);
	Ili9341_WriteCmd(0x11); 		//exit sleep
	HAL_Delay(50);

	Ili9341_WriteCmd(0xB6);
	Ili9341_WriteData(0x0A);
	Ili9341_WriteData(0xC2);

	Ili9341_WriteCmd(0x36);   	// memory access control
	Ili9341_WriteData(0x68);    // BGR -> seems RGB

	Ili9341_WriteCmd(0x3A); 	// pixel format
	Ili9341_WriteData(0x55); 	// RGB565 (16bit)

	Ili9341_WriteCmd(0xE0); 		//gamma
	Ili9341_WriteData(0x10);
  	Ili9341_WriteData(0x10);
  	Ili9341_WriteData(0x10);
  	Ili9341_WriteData(0x08);
  	Ili9341_WriteData(0x0E);
  	Ili9341_WriteData(0x06);
  	Ili9341_WriteData(0x42);
  	Ili9341_WriteData(0x28);
  	Ili9341_WriteData(0x36);
  	Ili9341_WriteData(0x03);
  	Ili9341_WriteData(0x0E);
  	Ili9341_WriteData(0x04);
  	Ili9341_WriteData(0x13);
  	Ili9341_WriteData(0x0E);
  	Ili9341_WriteData(0x0C);

  	Ili9341_WriteCmd(0XE1);
  	Ili9341_WriteData(0x0C);
  	Ili9341_WriteData(0x23);
  	Ili9341_WriteData(0x26);
  	Ili9341_WriteData(0x04);
  	Ili9341_WriteData(0x0C);
  	Ili9341_WriteData(0x04);
  	Ili9341_WriteData(0x39);
  	Ili9341_WriteData(0x24);
  	Ili9341_WriteData(0x4B);
  	Ili9341_WriteData(0x03);
  	Ili9341_WriteData(0x0B);
  	Ili9341_WriteData(0x0B);
  	Ili9341_WriteData(0x33);
  	Ili9341_WriteData(0x37);
  	Ili9341_WriteData(0x0F);

  	Ili9341_WriteCmd(0x2a);
  	Ili9341_WriteData(0x00);
  	Ili9341_WriteData(0x00);
  	Ili9341_WriteData(0x00);
  	Ili9341_WriteData(0xef);

  	Ili9341_WriteCmd(0x2b);
  	Ili9341_WriteData(0x00);
  	Ili9341_WriteData(0x00);
  	Ili9341_WriteData(0x01);
  	Ili9341_WriteData(0x3f);

  	Ili9341_WriteCmd(0x29);
  	HAL_Delay(10);
  	Ili9341_WriteCmd(0x2C);


  	Ili9341_DrawRect(0, 0, width, height, LCD_ILI9341_COLOR_BLACK);
  	Ili9341_SetArea(0, 0, width - 1, height - 1);

}



/*** Internal Function Defines ***/
void Ili9341_WriteCmd(uint16_t cmd)
{
	LCD_CMD = cmd;
}


void Ili9341_WriteData(uint16_t data)
{
	LCD_DATA = data;
}

inline static void Ili9341_ReadData()
{
	uint16_t data = LCD_DATA;
}





