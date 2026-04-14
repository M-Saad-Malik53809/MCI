#include "pid.h"

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->i_min = out_min;
    pid->i_max = out_max;

    pid->initialized = 1U;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(PID_Controller *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetIntegralLimits(PID_Controller *pid, float i_min, float i_max)
{
    pid->i_min = i_min;
    pid->i_max = i_max;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s)
{
    float error;
    float derivative;
    float output;

    if ((pid == 0) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    if (pid->initialized == 0U)
    {
        pid->prev_error = 0.0f;
        pid->integral = 0.0f;
        pid->initialized = 1U;
    }

    error = setpoint - measurement;
    pid->integral += error * dt_s;
    pid->integral = PID_Clamp(pid->integral, pid->i_min, pid->i_max);

    derivative = (error - pid->prev_error) / dt_s;

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    output = PID_Clamp(output, pid->out_min, pid->out_max);

    pid->prev_error = error;
    return output;
}
