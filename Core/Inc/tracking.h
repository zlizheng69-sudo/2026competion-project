#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/* 读取宏 — 引脚由 CubeMX 定义 (X1_Pin, X2_Pin, X3_Pin, X4_Pin) */
#define TRACK_X1_READ() !HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin)
#define TRACK_X2_READ() !HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin)
#define TRACK_X3_READ() !HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin)
#define TRACK_X4_READ() !HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin)

/* CubeMX MX_GPIO_Init 已初始化循迹引脚，此函数保留为空 */
void TRACK_IR4_Init(void);

#endif
