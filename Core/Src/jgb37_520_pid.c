#include "jgb37_520_pid.h"

#include <stddef.h>

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

bool jgb37_520_pid_init(jgb37_520_pid_t *pid,
                        float kp,
                        float ki,
                        float kd,
                        float output_min,
                        float output_max,
                        float integral_min,
                        float integral_max)
{
    if ((pid == NULL) ||
        (output_min > output_max) ||
        (integral_min > integral_max))
    {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    jgb37_520_pid_reset(pid);
    return true;
}

bool jgb37_520_pid_update(jgb37_520_pid_t *pid,
                          float setpoint,
                          float measurement,
                          float sample_period_s,
                          float *output)
{
    float derivative;
    float error;

    if ((pid == NULL) || (output == NULL) || (sample_period_s <= 0.0f))
    {
        return false;
    }

    error = setpoint - measurement;
    pid->integral = clamp_float(pid->integral + error * sample_period_s,
                                pid->integral_min,
                                pid->integral_max);

    if (pid->first_update)
    {
        derivative = 0.0f;
        pid->first_update = false;
    }
    else
    {
        derivative = (error - pid->previous_error) / sample_period_s;
    }

    *output = clamp_float(pid->kp * error +
                              pid->ki * pid->integral +
                              pid->kd * derivative,
                          pid->output_min,
                          pid->output_max);
    pid->previous_error = error;
    return true;
}

void jgb37_520_pid_reset(jgb37_520_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->first_update = true;
}
