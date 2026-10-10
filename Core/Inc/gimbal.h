#ifndef GIMBAL_H
#define GIMBAL_H

#include "pid.h"
#include "motor.h"

typedef struct {
    PID_Controller  pid;
    Motor_Instance  motor;
    float           target_angle;
    uint8_t         enabled;
} Gimbal_Axis;

void Gimbal_Init(Gimbal_Axis *ax, UART_HandleTypeDef *huart);
void Gimbal_Enable(Gimbal_Axis *ax, uint8_t en);
void Gimbal_SetTarget(Gimbal_Axis *ax, float angle_deg);
void Gimbal_SetGains(Gimbal_Axis *ax, float kp, float ki, float kd);
void Gimbal_Update(Gimbal_Axis *ax, float imu_angle, float gyro_rate, float dt);

#endif
