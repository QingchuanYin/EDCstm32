#ifndef JGB37_520_PID_H
#define JGB37_520_PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    bool first_update;
} jgb37_520_pid_t;

bool jgb37_520_pid_init(jgb37_520_pid_t *pid,
                        float kp,
                        float ki,
                        float kd,
                        float output_min,
                        float output_max,
                        float integral_min,
                        float integral_max);

bool jgb37_520_pid_update(jgb37_520_pid_t *pid,
                          float setpoint,
                          float measurement,
                          float sample_period_s,
                          float *output);

void jgb37_520_pid_reset(jgb37_520_pid_t *pid);

#endif /* JGB37_520_PID_H */
