#include "balance.h"
#include "SerialCmd.h"
#include <math.h>

#define BAL_KP          0.4f
#define BAL_KI          0.03f
#define BAL_KD          0.15f
#define BAL_OUT_LIMIT   130.0f
#define BAL_INT_LIMIT   100.0f
#define BAL_POS_LIMIT   2954

#define BRAKE_KV     0.5f
#define BRAKE_RANGE  2.0f

static Motor_Instance  mot;
static PID_Controller  pid;
static float           tgt;
static float           last_cm;
static uint8_t         on;
static float           total;
static float           pitch;
static float           ff;
static float           vel;
static float           kv = 0.1f;
static uint8_t         spd_mode;

static uint8_t  phase;
static uint32_t t0;
static uint8_t  wpi;
static uint8_t  done;

static float to_cm(int16_t raw)
{
    return (float)raw / 10.0f;
}

void Balance_SetSpeedMode(uint8_t en)
{
    spd_mode = en;
    if (!en) Motor_Stop(&mot);
}

static void send_out(float total_out)
{
    if (spd_mode) {
        if (fabsf(total_out) < 0.25f) { Motor_Stop(&mot); return; }
        float rpm = fabsf(total_out) * 0.9375f;
        if (rpm < 1.0f) rpm = 1.0f;
        if (rpm > 3000.0f) rpm = 3000.0f;
        uint8_t dir = (total_out > 0) ? 0x00 : 0x01;
        Motor_SetSpeed(&mot, dir, (uint16_t)rpm, 0, 0);
    } else {
        int32_t pulses = (int32_t)(total_out > 0 ? total_out + 0.5f : total_out - 0.5f);
        float pos_new = total + (float)pulses;
        if (pos_new >  BAL_POS_LIMIT) pulses = (int32_t)(BAL_POS_LIMIT - total);
        if (pos_new < -BAL_POS_LIMIT) pulses = (int32_t)(-BAL_POS_LIMIT - total);
        if (pulses == 0) return;
        total += (float)pulses;
        Motor_SetPosition(&mot, pulses);
    }
}

static float get_kv(float ball_cm, float target_cm)
{
    if (fabsf(ball_cm - target_cm) < BRAKE_RANGE)
        return BRAKE_KV;
    return kv;
}

void Balance_Init(UART_HandleTypeDef *huart)
{
    Motor_Init(&mot, 1, huart);
    PID_Init(&pid, BAL_KP, BAL_KI, BAL_KD,
             BAL_INT_LIMIT, BAL_OUT_LIMIT);
    tgt = BALL_CENTER_CM;
    on = 0;
    spd_mode = 0;
    phase = 0;
}

void Balance_Enable(uint8_t en)
{
    if (en == on) return;
    on = en;
    PID_Reset(&pid);
    total = 0.0f;
    pitch = 0.0f;
    phase = 0;
    wpi = 0;
    done = 0;

    Motor_Config cfg = { .speed = 100, .accel = 0, .mode = 2, .sync = 0 };

    if (en) {
        Motor_Enable(&mot, 1);
        Motor_SetConfig(&mot, &cfg);
        t0 = HAL_GetTick();
    } else {
        Motor_Enable(&mot, 0);
    }
}

void Balance_SetTarget(float cm)
{
    tgt = cm;
    PID_Reset(&pid);
}

void Balance_Update(int16_t ball_x, float dt)
{
    if (!on) return;

    float ball_cm = to_cm(ball_x);
    last_cm = ball_cm;

    float pid_out = PID_Update(&pid, tgt, ball_cm, dt);
    float kv_eff = get_kv(ball_cm, tgt);
    float ff_pulses = -(ff * pitch) - (kv_eff * vel);
    float total_out = -(pid_out + ff_pulses);

    send_out(total_out);
}

static const float wp_tgt[] = { 5.0f, -5.0f };
static const float wp_thr[] = { 0.5f, 0.3f };
#define WP_COUNT  2

uint8_t Balance_RunTrajectory(int16_t ball_x, float dt)
{
    if (!on) return 0;

    float ball_cm = to_cm(ball_x);
    last_cm = ball_cm;

    if (done) {
        tgt = -5.0f;
    } else {
        tgt = wp_tgt[wpi];
        if (fabsf(ball_cm - tgt) < wp_thr[wpi]) {
            if (wpi == WP_COUNT - 1) { done = 1; return 1; }
            wpi++;
        }
    }

    float pid_out = PID_Update(&pid, tgt, ball_cm, dt);
    float kv_eff = get_kv(ball_cm, tgt);
    float ff_pulses = -(ff * pitch) - (kv_eff * vel);
    float total_out = -(pid_out + ff_pulses);

    send_out(total_out);
    return 0;
}

void Balance_SetCarPitch(float deg)     { pitch = deg; }
float Balance_GetAbsAngle(void)         { return total * DEG_PER_PULSE + pitch; }
void Balance_SetFFGain(float k)         { if (k >= 0.0f) ff = k; }
float Balance_GetFFGain(void)           { return ff; }
void Balance_SetGains(float kp, float ki, float kd) { PID_SetGains(&pid, kp, ki, kd); }
float Balance_GetKp(void) { return pid.kp; }
float Balance_GetKi(void) { return pid.ki; }
float Balance_GetKd(void) { return pid.kd; }
void Balance_SetBallVel(float v)        { vel = v; }
void Balance_SetKv(float k)             { if (k >= 0.0f) kv = k; }
float Balance_GetKv(void)               { return kv; }

uint8_t Balance_IsStable(float threshold_cm)
{
    float error = tgt - last_cm;
    if (error < 0) error = -error;
    return (error < threshold_cm) ? 1 : 0;
}

