#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define PULSES_PER_REV     3200.0f
#define PULSES_PER_DEGREE  8.888889f
#define DEGREES_PER_PULSE  0.1125f

#define MOTOR_CHECKSUM      0x6B

typedef struct {
    uint16_t speed;       // RPM (0-3000)
    uint8_t  accel;       // 0-255
    uint8_t  mode;        // 0=relative-to-last  1=absolute  2=relative-to-current
    uint8_t  sync;        // 0=immediate  1=cached
} Motor_Config;

typedef struct {
    uint8_t                addr;
    uint8_t                enabled;
    UART_HandleTypeDef    *huart;
} Motor_Instance;

void Motor_Init(Motor_Instance *mot, uint8_t addr, UART_HandleTypeDef *huart);
int  Motor_SetConfig(Motor_Instance *mot, const Motor_Config *cfg);
int  Motor_Enable(Motor_Instance *mot, uint8_t enable);
int  Motor_SetPosition(Motor_Instance *mot, int32_t delta_pulses);
int  Motor_SetSpeed(Motor_Instance *mot, uint8_t direction, uint16_t speed_rpm, uint8_t accel, uint8_t sync);
int  Motor_Stop(Motor_Instance *mot);

#endif
