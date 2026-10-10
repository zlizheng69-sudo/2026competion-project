#include "IMU.h"
#include <stdio.h>
void IMU_getangle(ICM42688_Data *data, IMU_Angle *angle, float T,
                  float gx_offset, float gy_offset, float gz_offset)
{
    float dt = T / 1000.0f;
    float g_pitch, g_roll;
    float a_pitch, a_roll;

    // 1. 加速度计反算角度（只有 pitch 和 roll）
    // 注：调用者需先调用 ICM42688_ReadAct(data) 读取传感器数据
    a_pitch = atan2f(data->accel_y, data->accel_z) * 180.0f / PI;
    a_roll  = atan2f(-data->accel_x,
                     sqrtf(data->accel_y * data->accel_y +
                           data->accel_z * data->accel_z)) * 180.0f / PI;

    // 3. 陀螺仪增量（°/s × s = °），先扣除零偏
    g_pitch = (data->gyro_x - gx_offset) * dt;
    g_roll  = (data->gyro_y - gy_offset) * dt;

    // 4. 互补滤波：pitch / roll（陀螺仪积分 + 加速度计修正）
    angle->pitch = alpha * (angle->pitch + g_pitch) + (1.0f - alpha) * a_pitch;
    angle->roll  = alpha * (angle->roll  + g_roll)  + (1.0f - alpha) * a_roll;

    // 5. yaw（纯陀螺仪积分，死区抑制零漂）
    float gyro_z_corrected = data->gyro_z - gz_offset;
    if (gyro_z_corrected < YAW_DEADZONE && gyro_z_corrected > -YAW_DEADZONE)
        gyro_z_corrected = 0.0f;
    angle->gyro_z_f = beta * gyro_z_corrected + (1.0f - beta) * angle->gyro_z_f;
    angle->yaw += angle->gyro_z_f * dt;
}
