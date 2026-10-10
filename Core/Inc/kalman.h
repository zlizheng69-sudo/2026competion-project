#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

typedef struct {
    float px, vx;          // pixel position & velocity (yaw axis)
    float py, vy;          // pixel position & velocity (pitch axis)
    float P[4][4];         // covariance matrix
    uint32_t last_update_ms;
    uint32_t predict_count;
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf);
void Kalman_Predict(KalmanFilter *kf, float gyro_yaw_rad, float gyro_pitch_rad, float dt_s);
void Kalman_Update(KalmanFilter *kf, float dx_meas, float dy_meas);
void Kalman_GetAngles(KalmanFilter *kf, float *yaw_deg, float *pitch_deg);

#endif
