#include "jga25_370_pid.h"

void pid_init(pid_t *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}

float pid_compute(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* P — proportional to current error */
    float P = pid->Kp * error;

    /* I — accumulate error over time, then clamp to prevent windup.
     * Without clamping, the integrator grows unbounded when the output
     * saturates, causing slow recovery and overshoot when error clears. */
    pid->integral += error * dt;

    float i_term = pid->Ki * pid->integral;
    if (i_term > pid->output_max)
    {
        pid->integral = pid->output_max / pid->Ki;
        i_term        = pid->output_max;
    }
    else if (i_term < pid->output_min)
    {
        pid->integral = pid->output_min / pid->Ki;
        i_term        = pid->output_min;
    }

    /* D — rate of change of error, dampens overshoot */
    float derivative = (error - pid->prev_error) / dt;
    float D          = pid->Kd * derivative;

    pid->prev_error = error;

    /* Sum all three terms and clamp to output limits */
    float output = P + i_term + D;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}
