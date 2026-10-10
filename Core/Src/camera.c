#include "camera.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>

static uint8_t  rx_state;
static int8_t   rx_pos;
static char     rx_det;
static CameraData data;
static uint32_t byte_cnt;
static uint32_t packet_cnt;
static uint32_t error_cnt;

void Camera_Init(void)
{
    rx_state = 0;
    data.dx = 0;
    data.dy = 0;
    data.detected = 0;
    data.fresh = 0;
    packet_cnt = 0;
    error_cnt = 0;

    HAL_NVIC_SetPriority(USART3_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void Camera_RxByte(uint8_t byte)
{
    byte_cnt++;
    switch (rx_state) {
    case 0:
        if (byte == 0xAA) {
            rx_state = 1;
        }
        break;
    case 1:
        rx_pos = (int8_t)byte;
        rx_state = 2;
        break;
    case 2:
        rx_det = (char)byte;
        rx_state = 3;
        break;
    case 3:
    {
        uint8_t cksum = (uint8_t)(0xAA + rx_pos + rx_det);
        if (byte == cksum) {
            data.dx = (int16_t)rx_pos;
            data.detected = (rx_det == 'Y') ? 1 : 0;
            data.fresh = 1;
            packet_cnt++;
        } else {
            error_cnt++;
        }
        rx_state = 0;
        break;
    }
    default:
        rx_state = 0;
        break;
    }
}

CameraData Camera_GetData(void)
{
    CameraData out = data;
    data.fresh = 0;
    return out;
}

CameraData Camera_PeekData(void)
{
    return data;
}

void Camera_GetStats(uint32_t *pkt, uint32_t *err, uint32_t *bytes)
{
    *pkt = packet_cnt;
    *err = error_cnt;
    *bytes = byte_cnt;
}
