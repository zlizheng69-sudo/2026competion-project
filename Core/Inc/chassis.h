#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "tim.h"
#include "tracking.h"

/* 电机编号 */
#define CHASSIS_MOTOR_L  0
#define CHASSIS_MOTOR_R  1

/* TB6612 左轮 AIN1/AIN2 — PC7/PC6 */
#define AIN1_PIN    GPIO_PIN_7
#define AIN1_PORT   GPIOC
#define AIN2_PIN    GPIO_PIN_6
#define AIN2_PORT   GPIOC

/* TB6612 右轮 BIN1/BIN2 — PB0/PB1 */
#define BIN1_PIN    GPIO_PIN_0
#define BIN1_PORT   GPIOB
#define BIN2_PIN    GPIO_PIN_1
#define BIN2_PORT   GPIOB

/* TB6612 STBY — PD15 */
#define STBY_PIN    GPIO_PIN_14
#define STBY_PORT   GPIOD

/* 最大 PWM 比较值 (ARR=23999) */
#define CHASSIS_PWM_MAX   23999.0f

void Chassis_Init(void);
void Chassis_SetMotor(uint8_t motor, float duty);
void Chassis_SetMotorRaw(uint8_t motor, float duty);  /* 绕过滤波，速度环专用 */
void Chassis_Stop(void);
void Chassis_SoftBrake(void);
void Chassis_BrakeHard(void);
uint8_t Chassis_BrakeUpdate(void);
void Chassis_LineFollow(void);
void Chassis_LineFollowAdv(void);
void Chassis_LineRatio(float *rL, float *rR);

#endif
