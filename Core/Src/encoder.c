#include "encoder.h"

static int32_t last[2], dlt[2];
static float   spd[2];
static int32_t dist[2];

void Enc_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    last[0] = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    last[1] = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    dlt[0] = dlt[1] = 0;
    spd[0] = spd[1] = 0.0f;
    dist[0] = dist[1] = 0;
}

int32_t Enc_Cnt(uint8_t n)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(n == ENC_LEFT ? &htim2 : &htim3);
}

void Enc_Update(void)
{
    int32_t nL = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t nR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    dlt[0] = nL - last[0]; dlt[1] = nR - last[1];
    last[0] = nL; last[1] = nR;
    spd[0] = (float)dlt[0]; spd[1] = (float)dlt[1];
    dist[0] += dlt[0] >= 0 ? dlt[0] : -dlt[0];
    dist[1] += dlt[1] >= 0 ? dlt[1] : -dlt[1];
}

float Enc_Spd(uint8_t n)  { return spd[n]; }
int32_t Enc_Delta(uint8_t n) { return dlt[n]; }

float Enc_Dist(void)
{
    return (float)(dist[0] + dist[1]) * 0.5f * MM_PER_CNT;
}

void Enc_ResetDist(void)
{
    dist[0] = dist[1] = 0;
    last[0] = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    last[1] = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    spd[0] = spd[1] = 0.0f;
}
