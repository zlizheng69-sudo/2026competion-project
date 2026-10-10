#include "kalman.h"
#include <string.h>
#include <math.h>

/* ── Document parameters (section 6.3 / 6.4) ── */
#define IMU_BUF_LEN     70u    // covers ~140 ms at 500 Hz
#define T_LATENCY_MS    50u    // visual pipeline delay (default, to be measured)
#define FX_PIXEL        1412.0f // equiv. focal length, pixels/rad
#define FY_PIXEL        1412.0f

#define Q_PX   25.0f
#define Q_VX   2500.0f
#define Q_PY   25.0f
#define Q_VY   2500.0f

#define R_DX   9.0f
#define R_DY   9.0f

#define P0_PX  10000.0f
#define P0_VX  1000.0f
#define P0_PY  10000.0f
#define P0_VY  1000.0f

#define POS_LIMIT    200.0f   // px
#define VEL_LIMIT    200.0f   // px/s
#define TIMEOUT_MS   200u      // no-measurement timeout
#define TAU_DECAY    0.5f     // decay time constant (s)

/* ── IMU ring buffer ── */
static float  imu_yaw_buf[IMU_BUF_LEN];
static float  imu_pitch_buf[IMU_BUF_LEN];
static int    imu_wr_idx;

/* ── helpers ── */
static inline void buf_store(float yaw_rad, float pitch_rad)
{
    imu_yaw_buf[imu_wr_idx]   = yaw_rad;
    imu_pitch_buf[imu_wr_idx] = pitch_rad;
    imu_wr_idx = (imu_wr_idx + 1) % IMU_BUF_LEN;
}

static inline float buf_sum_yaw(uint32_t n)
{
    float s = 0.0f;
    int start = (imu_wr_idx - (int)n + IMU_BUF_LEN) % IMU_BUF_LEN;
    for (uint32_t i = 0; i < n; i++)
        s += imu_yaw_buf[(start + i) % IMU_BUF_LEN];
    return s;
}

static inline float buf_sum_pitch(uint32_t n)
{
    float s = 0.0f;
    int start = (imu_wr_idx - (int)n + IMU_BUF_LEN) % IMU_BUF_LEN;
    for (uint32_t i = 0; i < n; i++)
        s += imu_pitch_buf[(start + i) % IMU_BUF_LEN];
    return s;
}

void Kalman_Init(KalmanFilter *kf)
{
    memset(kf, 0, sizeof(*kf));
    kf->P[0][0] = P0_PX;  kf->P[1][1] = P0_VX;
    kf->P[2][2] = P0_PY;  kf->P[3][3] = P0_VY;
    memset(imu_yaw_buf,   0, sizeof(imu_yaw_buf));
    memset(imu_pitch_buf, 0, sizeof(imu_pitch_buf));
    imu_wr_idx = 0;
}

/* ── Predict step (called at control-loop rate, ~500 Hz) ── */
void Kalman_Predict(KalmanFilter *kf, float gyro_yaw_rad, float gyro_pitch_rad, float dt_s)
{
    /* --- store IMU sample for measurement propagation --- */
    buf_store(gyro_yaw_rad, gyro_pitch_rad);

    float dt = dt_s;
    if (dt <= 0.0f) return;

    /* --- state transition --- */
    kf->px += kf->vx * dt + FX_PIXEL * gyro_yaw_rad * dt;
    kf->py += kf->vy * dt + FY_PIXEL * gyro_pitch_rad * dt;

    /* --- P prediction:  P = F·P·F^T + Q  (explicit for sparse F) --- */
    // F = [[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]]
    // Compute F·P temporarily in-place via explicit formulas,
    // then multiply by F^T, then add Q.

    float tmp[4][4];
    for (int r = 0; r < 4; r++) {
        tmp[r][0] = kf->P[r][0] + dt * kf->P[r][1];
        tmp[r][1] = kf->P[r][1];
        tmp[r][2] = kf->P[r][2] + dt * kf->P[r][3];
        tmp[r][3] = kf->P[r][3];
    }

    // (F·P)·F^T
    for (int r = 0; r < 4; r++) {
        kf->P[r][0] = tmp[r][0] + dt * tmp[r][1];  // col0 += dt*col1
        kf->P[r][1] = tmp[r][1];                    // col1 unchanged
        kf->P[r][2] = tmp[r][2] + dt * tmp[r][3];  // col2 += dt*col3
        kf->P[r][3] = tmp[r][3];                    // col3 unchanged
    }

    // + Q
    kf->P[0][0] += Q_PX;
    kf->P[1][1] += Q_VX;
    kf->P[2][2] += Q_PY;
    kf->P[3][3] += Q_VY;

    kf->predict_count++;
}

/* ── Update step with measurement propagation (§6.1) ── */
void Kalman_Update(KalmanFilter *kf, float dx_meas, float dy_meas)
{
    /* --- measurement propagation --- */
    uint32_t n = T_LATENCY_MS;
    if (n > IMU_BUF_LEN) n = IMU_BUF_LEN;
    float sum_yaw   = buf_sum_yaw(n);
    float sum_pitch = buf_sum_pitch(n);
    float T_s       = (float)T_LATENCY_MS * 0.001f;

    float dx_prop = dx_meas + kf->vx * T_s + FX_PIXEL * sum_yaw   * 0.001f;
    float dy_prop = dy_meas + kf->vy * T_s + FY_PIXEL * sum_pitch * 0.001f;

    /* --- innovation --- */
    float y0 = dx_prop - kf->px;
    float y1 = dy_prop - kf->py;

    /* --- S = H·P·H^T + R   (2×2) --- */
    float S00 = kf->P[0][0] + R_DX;
    float S01 = kf->P[0][2];
    float S10 = kf->P[2][0];
    float S11 = kf->P[2][2] + R_DY;

    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-9f) return;

    float inv00 =  S11 / det;
    float inv01 = -S01 / det;
    float inv10 = -S10 / det;
    float inv11 =  S00 / det;

    /* --- K = P·H^T · S^{-1}   (4×2) --- */
    // P·H^T = [col0, col2] of P
    float K00 = kf->P[0][0]*inv00 + kf->P[0][2]*inv10;
    float K01 = kf->P[0][0]*inv01 + kf->P[0][2]*inv11;
    float K10 = kf->P[1][0]*inv00 + kf->P[1][2]*inv10;
    float K11 = kf->P[1][0]*inv01 + kf->P[1][2]*inv11;
    float K20 = kf->P[2][0]*inv00 + kf->P[2][2]*inv10;
    float K21 = kf->P[2][0]*inv01 + kf->P[2][2]*inv11;
    float K30 = kf->P[3][0]*inv00 + kf->P[3][2]*inv10;
    float K31 = kf->P[3][0]*inv01 + kf->P[3][2]*inv11;

    /* --- state update  x = x + K·y --- */
    kf->px += K00 * y0 + K01 * y1;
    kf->vx += K10 * y0 + K11 * y1;
    kf->py += K20 * y0 + K21 * y1;
    kf->vy += K30 * y0 + K31 * y1;

    /* --- P update  P = (I - K·H)·P -------------------------------
       (I - K·H)[i][j] = I[i][j] - K[i][0]*H[0][j] - K[i][1]*H[1][j]
       H[0]=[1,0,0,0]  H[1]=[0,0,1,0]  → only cols 0,2 affected
    --------------------------------------------------------------- */
    for (int i = 0; i < 4; i++) {
        float c0 = kf->P[i][0];
        float c1 = kf->P[i][1];
        float c2 = kf->P[i][2];
        float c3 = kf->P[i][3];
        kf->P[i][0] = c0 - (K00 * kf->P[0][0] + K01 * kf->P[2][0]);
        kf->P[i][1] = c1 - (K00 * kf->P[0][1] + K01 * kf->P[2][1]);
        kf->P[i][2] = c2 - (K00 * kf->P[0][2] + K01 * kf->P[2][2]);
        kf->P[i][3] = c3 - (K00 * kf->P[0][3] + K01 * kf->P[2][3]);
    }

    /* --- anti-windup clamping (§6.6) --- */
    if (kf->px >  POS_LIMIT) kf->px =  POS_LIMIT;
    if (kf->px < -POS_LIMIT) kf->px = -POS_LIMIT;
    if (kf->py >  POS_LIMIT) kf->py =  POS_LIMIT;
    if (kf->py < -POS_LIMIT) kf->py = -POS_LIMIT;
    if (kf->vx >  VEL_LIMIT) kf->vx =  VEL_LIMIT;
    if (kf->vx < -VEL_LIMIT) kf->vx = -VEL_LIMIT;
    if (kf->vy >  VEL_LIMIT) kf->vy =  VEL_LIMIT;
    if (kf->vy < -VEL_LIMIT) kf->vy = -VEL_LIMIT;

    /* --- P symmetry (every 100 updates) --- */
    if (kf->predict_count % 100 == 0) {
        kf->P[0][1] = kf->P[1][0] = (kf->P[0][1] + kf->P[1][0]) * 0.5f;
        kf->P[0][2] = kf->P[2][0] = (kf->P[0][2] + kf->P[2][0]) * 0.5f;
        kf->P[0][3] = kf->P[3][0] = (kf->P[0][3] + kf->P[3][0]) * 0.5f;
        kf->P[1][2] = kf->P[2][1] = (kf->P[1][2] + kf->P[2][1]) * 0.5f;
        kf->P[1][3] = kf->P[3][1] = (kf->P[1][3] + kf->P[3][1]) * 0.5f;
        kf->P[2][3] = kf->P[3][2] = (kf->P[2][3] + kf->P[3][2]) * 0.5f;
    }

    kf->last_update_ms = 0;  // updated by caller via HAL_GetTick
}

/* ── Output (§6.5 Step 4) ── */
void Kalman_GetAngles(KalmanFilter *kf, float *yaw_deg, float *pitch_deg)
{
    // 1/fx ≈ 0.000708 rad/px, × 180/π ≈ 0.04058 °/px
    *yaw_deg   = kf->px * 0.04058f;
    *pitch_deg = kf->py * 0.04058f;
}
