#ifndef SERIALCMD_H
#define SERIALCMD_H

#include "gimbal.h"
#include "pid.h"
#include <stdint.h>

/* 全局状态变量 */
extern volatile uint8_t  g_task_mode;
extern volatile float    g_ball_target;
extern volatile uint8_t  g_key_pressed;
extern volatile uint32_t g_lap_time_ms;


void SerialCmd_Init(void);
void SerialCmd_Update(Gimbal_Axis *g, PID_Controller *vp);
void SerialCmd_RxByte(uint8_t byte);

#endif
