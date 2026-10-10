#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include <stdint.h>
#include "pid.h"

#define SPD_CRUISE  11.0f

#define SPD_CALIB_L   21.6f
#define SPD_CALIB_R   23.3f
#define SPD_CALIB_DTY 0.5f

#define SPD_FF_L  (SPD_CALIB_L / SPD_CALIB_DTY)
#define SPD_FF_R  (SPD_CALIB_R / SPD_CALIB_DTY)

#define POS_KP_DEFAULT  0.01f
#define POS_MIN_DECEL   0.1f

void Spd_Init(void);
void Spd_Update(void);
void Spd_SetRatio(float L, float R);
void Spd_SetStop(float mm);
uint8_t Spd_Arrived(void);
float Spd_Remain(void);
void Spd_Reset(void);
PID_Controller* Spd_GetPidL(void);
PID_Controller* Spd_GetPidR(void);
float Spd_GetPosKp(void);
void Spd_SetPosKp(float kp);
void Spd_Direct(uint8_t on);
uint8_t Spd_IsDirect(void);
void Spd_SetDuty(float d);
void Spd_AddDist(float mm);

#endif
