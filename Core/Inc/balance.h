#ifndef __BALANCE_H
#define __BALANCE_H

#include "main.h"
#include "pid.h"
#include "motor.h"

#define GEAR_PITCH_DIAM_MM   39.0f
#define GEAR_RADIUS_MM       19.5f
#define SWING_ARM_MM         250.0f
#define RACK_ARM_MM          225.0f
#define PULSES_PER_REV       3200.0f

#define DEG_PER_PULSE  (360.0f / PULSES_PER_REV * GEAR_RADIUS_MM / RACK_ARM_MM)

#define GROOVE_LENGTH_CM   25.0f
#define BALL_CENTER_CM     0.0f

void Balance_Init(UART_HandleTypeDef *huart);
void Balance_Update(int16_t ball_x, float dt);
uint8_t Balance_RunTrajectory(int16_t ball_x, float dt);
void Balance_SetTarget(float cm);
void Balance_Enable(uint8_t en);
uint8_t Balance_IsStable(float threshold_cm);
void Balance_SetCarPitch(float deg);
float Balance_GetAbsAngle(void);
void Balance_SetFFGain(float k);
float Balance_GetFFGain(void);
void Balance_SetGains(float kp, float ki, float kd);
void Balance_SetSpeedMode(uint8_t en);
void Balance_SetBallVel(float vel_cm_s);
void Balance_SetKv(float k);
float Balance_GetKv(void);
float Balance_GetKp(void);
float Balance_GetKi(void);
float Balance_GetKd(void);

#endif
