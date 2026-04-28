#ifndef PID_H
#define PID_H
#ifdef __cplusplus
extern "C" {
#endif

#define RIGHT_PWM_CHANNEL       TIM_CHANNEL_1
#define LEFT_PWM_CHANNEL        TIM_CHANNEL_2

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float out_min;
    float out_max;

    float i_min;
    float i_max;

    unsigned char initialized;
} PID_Controller;

void PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max);

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetOutputLimits(PID_Controller *pid, float out_min, float out_max);
void PID_SetIntegralLimits(PID_Controller *pid, float i_min, float i_max);
void PID_Reset(PID_Controller *pid);

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
