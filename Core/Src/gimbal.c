#include "gimbal.h"
#include <math.h>

#define DEADBAND_DEG   0.05f
#define DEFAULT_KP     2.0f
#define DEFAULT_KI     0.0f
#define DEFAULT_KD     0.5f
#define INTEGRAL_LIMIT 5.0f
#define OUTPUT_LIMIT   20.0f

void Gimbal_Init(Gimbal_Axis *ax, UART_HandleTypeDef *huart)
{
    Motor_Init(&ax->motor, 1, huart);
    PID_Init(&ax->pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             INTEGRAL_LIMIT, OUTPUT_LIMIT);
    ax->target_angle = 0.0f;
    ax->enabled = 0;
}

void Gimbal_Enable(Gimbal_Axis *ax, uint8_t en)
{
    if (en == ax->enabled) return;
    ax->enabled = en;
    PID_Reset(&ax->pid);

    Motor_Config cfg = {
        .speed = 1500,
        .accel = 80,
        .mode  = 2,
        .sync  = 0,
    };

    if (en) {
        Motor_Enable(&ax->motor, 1);
        Motor_SetConfig(&ax->motor, &cfg);
    } else {
        Motor_Enable(&ax->motor, 0);
    }
}

void Gimbal_SetTarget(Gimbal_Axis *ax, float angle_deg)
{
    ax->target_angle = angle_deg;
}

void Gimbal_SetGains(Gimbal_Axis *ax, float kp, float ki, float kd)
{
    PID_SetGains(&ax->pid, kp, ki, kd);
}

void Gimbal_Update(Gimbal_Axis *ax, float imu_angle, float gyro_rate, float dt)
{
    if (!ax->enabled) return;

    float pid_out = PID_Update(&ax->pid, ax->target_angle, imu_angle, dt);

    float ff_deg = -gyro_rate * dt;
    float total_deg = pid_out + ff_deg;

    if (fabsf(total_deg) < DEADBAND_DEG) return;

    int32_t pulses = (int32_t)(total_deg * PULSES_PER_DEGREE);
    Motor_SetPosition(&ax->motor, pulses);
}
