#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_measurement;
    float integral_limit;
    float output_limit;
    float d_filtered;
    float d_tau;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_Controller *pid);
void PID_SetGains(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetDTau(PID_Controller *pid, float tau);
void PID_ResetDerivative(PID_Controller *pid);

#endif
