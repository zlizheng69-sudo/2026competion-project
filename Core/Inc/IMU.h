#ifndef IMU_H
#define IMU_H

#include "ICM42688.h"
#include "math.h"
#define alpha 0.966
#define beta 0.15f
#define YAW_DEADZONE 0.5f   // yaw死区 (dps)，低于此值的角速度视为零漂
#define PI 3.1415926
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float gyro_z_f;     // 低通滤波后的陀螺仪 z 值
} IMU_Angle;
void IMU_getangle(ICM42688_Data *data,IMU_Angle *angle,float T,float gx_offset, float gy_offset, float gz_offset);

#endif /* IMU_H */