/*
 * lcdIli9341.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef LCDILI9341_LCDILI9341_H_
#define LCDILI9341_LCDILI9341_H_


#define FSMC_Ax               16              // use A16 as RS
#define FSMC_NEx              1               // use subbank 1

#define LCD_ILI9341_WIDTH  320
#define LCD_ILI9341_HEIGHT 240

void Ili9341_WriteData(uint16_t data);
void Ili9341_WriteCmd (uint16_t cmd);

void Ili9341_Init(uint16_t width, uint16_t height);
void Ili9341_SetArea(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void Ili9341_SetAreaRead(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void Ili9341_DrawRect(uint16_t xStart, uint16_t yStart, uint16_t width, uint16_t height, uint16_t color);
uint16_t* Ili9341_GetDrawAddress();


/* --------------------------------RGB565 -------------------------------------*/
#define LCD_ILI9341_COLOR_BLUE          0x001F
#define LCD_ILI9341_COLOR_GREEN         0x07E0
#define LCD_ILI9341_COLOR_RED           0xF800
#define LCD_ILI9341_COLOR_CYAN          0x07FF
#define LCD_ILI9341_COLOR_MAGENTA       0xF81F
#define LCD_ILI9341_COLOR_YELLOW        0xFFE0
#define LCD_ILI9341_COLOR_LIGHTBLUE     0x841F
#define LCD_ILI9341_COLOR_LIGHTGREEN    0x87F0
#define LCD_ILI9341_COLOR_LIGHTRED      0xFC10
#define LCD_ILI9341_COLOR_LIGHTCYAN     0x87FF
#define LCD_ILI9341_COLOR_LIGHTMAGENTA  0xFC1F
#define LCD_ILI9341_COLOR_LIGHTYELLOW   0xFFF0
#define LCD_ILI9341_COLOR_DARKBLUE      0x0010
#define LCD_ILI9341_COLOR_DARKGREEN     0x0400
#define LCD_ILI9341_COLOR_DARKRED       0x8000
#define LCD_ILI9341_COLOR_DARKCYAN      0x0410
#define LCD_ILI9341_COLOR_DARKMAGENTA   0x8010
#define LCD_ILI9341_COLOR_DARKYELLOW    0x8400
#define LCD_ILI9341_COLOR_WHITE         0xFFFF
#define LCD_ILI9341_COLOR_LIGHTGRAY     0xD69A
#define LCD_ILI9341_COLOR_GRAY          0x8410
#define LCD_ILI9341_COLOR_DARKGRAY      0x4208
#define LCD_ILI9341_COLOR_BLACK         0x0000
#define LCD_ILI9341_COLOR_BROWN         0xA145
#define LCD_ILI9341_COLOR_ORANGE        0xFD20



#endif /* LCDILI9341_LCDILI9341_H_ */
