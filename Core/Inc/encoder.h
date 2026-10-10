#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h"

#define ENC_LEFT   0
#define ENC_RIGHT  1
#define ENC_DT_MS  10.0f

#define WHEEL_DIA     66.0f
#define ENC_PPR       11
#define GEAR_RATIO    18.8f
#define QUAD_MUL      4
#define CNT_PER_REV   (ENC_PPR * QUAD_MUL * GEAR_RATIO)
#define MM_PER_CNT    (3.14159265f * WHEEL_DIA / CNT_PER_REV)

void Enc_Init(void);
int32_t Enc_Cnt(uint8_t n);
void Enc_Update(void);
float Enc_Spd(uint8_t n);
int32_t Enc_Delta(uint8_t n);
float Enc_Dist(void);
void Enc_ResetDist(void);

#endif
