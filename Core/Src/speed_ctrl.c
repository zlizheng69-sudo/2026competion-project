#include "speed_ctrl.h"
#include "encoder.h"
#include "pid.h"
#include "chassis.h"

#define SPD_KP   0.224f
#define SPD_KI   0.00f
#define SPD_KD   0.1f
#define SPD_OLIM 1.0f
#define SPD_ILIM 10.0f

static PID_Controller pid_L, pid_R;
static float rL = 1.0f, rR = 1.0f;
static float stop_mm, pos_kp = POS_KP_DEFAULT;
static float fduty = 0.30f, doff = 0.0f;
static float flt_L, flt_R;
static uint8_t stop_on, arrived, on, dir;

void Spd_Init(void)
{
    PID_Init(&pid_L, SPD_KP, SPD_KI, SPD_KD, SPD_ILIM, SPD_OLIM);
    PID_Init(&pid_R, SPD_KP, SPD_KI, SPD_KD, SPD_ILIM, SPD_OLIM);
    rL = 1.0f; rR = 1.0f;
    stop_mm = 0.0f; pos_kp = POS_KP_DEFAULT;
    fduty = 0.30f; doff = 0.0f;
    flt_L = 0.0f; flt_R = 0.0f;
    dir = 1; stop_on = 0; arrived = 0; on = 0;
}

void Spd_SetRatio(float L, float R)  { rL = L; rR = R; }

void Spd_SetStop(float mm)
{
    stop_mm = mm; stop_on = 1; arrived = 0;
    Enc_ResetDist();
}

void Spd_Reset(void)
{
    PID_Reset(&pid_L); PID_Reset(&pid_R);
    Enc_ResetDist();
    flt_L = 0.0f; flt_R = 0.0f;
    stop_on = 0; arrived = 0; on = 1; dir = 1;
}

uint8_t Spd_Arrived(void) { return arrived; }

float Spd_Remain(void)
{
    if (!stop_on) return 9999.0f;
    float r = stop_mm - Enc_Dist() + doff;
    return r > 0.0f ? r : 0.0f;
}

void Spd_Update(void)
{
    if (!on) return;

    Enc_Update();
    float a = 0.3f;
    flt_L = a * Enc_Spd(0) + (1.0f - a) * flt_L;
    flt_R = a * Enc_Spd(1) + (1.0f - a) * flt_R;

    float ds = 1.0f;
    if (stop_on) {
        float rem = stop_mm - Enc_Dist() + doff;
        if (rem <= 0.0f) {
            Chassis_Stop();
            arrived = 1; on = 0;
            return;
        }
        ds = pos_kp * rem;
        if (ds > 1.0f) ds = 1.0f;
        if (ds < POS_MIN_DECEL) ds = POS_MIN_DECEL;
    }

    if (dir) {
        float dl = ds * fduty * rL;
        float dr = ds * fduty * rR;
        if (dl > 1.0f) dl = 1.0f;
        if (dr > 1.0f) dr = 1.0f;
        Chassis_SetMotorRaw(0, dl);
        Chassis_SetMotorRaw(1, dr);
        return;
    }

    float base = SPD_CRUISE * ds;
    float tl = base * rL, tr = base * rR;
    float ffL = tl / SPD_FF_L, ffR = tr / SPD_FF_R;
    float cl = PID_Update(&pid_L, tl, flt_L, 0.01f);
    float cr = PID_Update(&pid_R, tr, flt_R, 0.01f);
    float dl = ffL + cl, dr = ffR + cr;
    if (dl > 1.0f) dl = 1.0f;
    if (dl < -1.0f) dl = -1.0f;
    if (dr > 1.0f) dr = 1.0f;
    if (dr < -1.0f) dr = -1.0f;
    Chassis_SetMotorRaw(0, dl);
    Chassis_SetMotorRaw(1, dr);
}

PID_Controller* Spd_GetPidL(void) { return &pid_L; }
PID_Controller* Spd_GetPidR(void) { return &pid_R; }
float Spd_GetPosKp(void) { return pos_kp; }
void Spd_SetPosKp(float k) { if (k >= 0.0f) pos_kp = k; }
void Spd_Direct(uint8_t d)  { dir = d; }
uint8_t Spd_IsDirect(void)  { return dir; }
void Spd_SetDuty(float d)   { if (d > 0.0f && d <= 1.0f) fduty = d; }
void Spd_AddDist(float mm)  { doff = mm; }
