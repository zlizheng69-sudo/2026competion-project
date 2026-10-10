#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->d_filtered = 0.0f;
    pid->d_tau = 0.2f;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    float p_out = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float i_out = pid->ki * pid->integral;

    float d_out = 0.0f;
    if (dt > 1e-6f) {
        float d_raw = -(measurement - pid->prev_measurement) / dt;
        if (pid->d_tau > 0.0f) {
            float alpha = dt / (pid->d_tau + dt);
            pid->d_filtered = alpha * d_raw + (1.0f - alpha) * pid->d_filtered;
            d_out = pid->kd * pid->d_filtered;
        } else {
            d_out = pid->kd * d_raw;
        }
    }
    pid->prev_measurement = measurement;

    float output = p_out + i_out + d_out;

    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
}

void PID_ResetDerivative(PID_Controller *pid)
{
    pid->d_filtered = 0.0f;
}

void PID_SetGains(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetDTau(PID_Controller *pid, float tau)
{
    pid->d_tau = tau;
    pid->d_filtered = 0.0f;
}
