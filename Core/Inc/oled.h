#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* SSD1306 分辨率 */
#define OLED_WIDTH   128
#define OLED_HEIGHT  64

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen);
void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...);

#endif
