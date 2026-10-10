#include "motor.h"

static void write_u16_be(uint8_t *buf, uint16_t val)
{
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

static void write_i32_be(uint8_t *buf, int32_t val)
{
    uint32_t u = (uint32_t)val;
    buf[0] = (u >> 24) & 0xFF;
    buf[1] = (u >> 16) & 0xFF;
    buf[2] = (u >> 8) & 0xFF;
    buf[3] = u & 0xFF;
}

void Motor_Init(Motor_Instance *mot, uint8_t addr, UART_HandleTypeDef *huart)
{
    mot->addr = addr;
    mot->enabled = 0;
    mot->huart = huart;
}

int Motor_SetConfig(Motor_Instance *mot, const Motor_Config *cfg)
{
    uint8_t buf[8];
    buf[0] = mot->addr;
    buf[1] = 0xF1;
    write_u16_be(&buf[2], cfg->speed);
    buf[4] = cfg->accel;
    buf[5] = cfg->mode;
    buf[6] = cfg->sync;
    buf[7] = MOTOR_CHECKSUM;

    if (HAL_UART_Transmit(mot->huart, buf, 8, 10) != HAL_OK)
        return -1;
    return 0;
}

int Motor_Enable(Motor_Instance *mot, uint8_t enable)
{
    uint8_t buf[6];
    buf[0] = mot->addr;
    buf[1] = 0xF3;
    buf[2] = 0xAB;
    buf[3] = enable ? 0x01 : 0x00;
    buf[4] = 0x00;
    buf[5] = MOTOR_CHECKSUM;

    if (HAL_UART_Transmit(mot->huart, buf, 6, 10) != HAL_OK)
        return -1;

    mot->enabled = enable;
    return 0;
}

int Motor_SetPosition(Motor_Instance *mot, int32_t delta_pulses)
{
    uint8_t buf[7];
    buf[0] = mot->addr;
    buf[1] = 0xFC;
    write_i32_be(&buf[2], delta_pulses);
    buf[6] = MOTOR_CHECKSUM;

    if (HAL_UART_Transmit(mot->huart, buf, 7, 10) != HAL_OK)
        return -1;
    return 0;
}

int Motor_SetSpeed(Motor_Instance *mot, uint8_t direction, uint16_t speed_rpm, uint8_t accel, uint8_t sync)
{
    uint8_t buf[8];
    buf[0] = mot->addr;
    buf[1] = 0xF6;
    buf[2] = direction;
    write_u16_be(&buf[3], speed_rpm);
    buf[5] = accel;
    buf[6] = sync;
    buf[7] = MOTOR_CHECKSUM;

    if (HAL_UART_Transmit(mot->huart, buf, 8, 10) != HAL_OK)
        return -1;
    return 0;
}

int Motor_Stop(Motor_Instance *mot)
{
    uint8_t buf[5];
    buf[0] = mot->addr;
    buf[1] = 0xFE;
    buf[2] = 0x98;
    buf[3] = 0x00;
    buf[4] = MOTOR_CHECKSUM;

    if (HAL_UART_Transmit(mot->huart, buf, 5, 10) != HAL_OK)
        return -1;
    return 0;
}
