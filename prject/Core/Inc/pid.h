#ifndef PID_H
#define PID_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Platform and control constants kept here for single-point tuning. */
/* CHR-GM37-520 nominal output-shaft PPR: 11 pulses * 30:1 gearbox = 330 */
#define PPR                     330.0f

// Self-balancing friendly defaults: fast PWM and split-rate loops.
#define PWM_FREQ_HZ             20000U
#define PWM_PERIOD_TICKS        2399U   // 48 MHz / 20 kHz - 1
#define BALANCE_LOOP_HZ         200.0f  // inner angle loop target
#define SPEED_LOOP_HZ           100.0f  // wheel PID loop target
#define SPEED_DT_S              (1.0f / SPEED_LOOP_HZ)

#define RIGHT_PWM_CHANNEL       TIM_CHANNEL_1
#define LEFT_PWM_CHANNEL        TIM_CHANNEL_2

// Central PID tuning constants for wheel-speed control.
// Tune these values in one place during bring-up.
#define PID_LEFT_KP            0.0f
#define PID_LEFT_KI            0.0f
#define PID_LEFT_KD            0.0f

#define PID_RIGHT_KP           0.0f
#define PID_RIGHT_KI           0.0f
#define PID_RIGHT_KD           0.0f

// Command/output limits. Use signed range for forward/reverse drive command.
#define PID_OUTPUT_MIN         -2399.0f
#define PID_OUTPUT_MAX         2399.0f

// Integral clamp (anti-windup). Keep tighter than output while tuning.
#define PID_INTEGRAL_MIN       -1200.0f
#define PID_INTEGRAL_MAX       1200.0f

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
