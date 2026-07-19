#ifndef JGA25_370_PID_H
#define JGA25_370_PID_H

/* PID controller state and tuning parameters */
typedef struct
{
    float Kp;           /* proportional gain                                   */
    float Ki;           /* integral gain                                       */
    float Kd;           /* derivative gain                                     */

    float prev_error;   /* error from previous cycle — used by derivative term */
    float integral;     /* accumulated integral term                           */

    float output_min;   /* minimum allowed output value                        */
    float output_max;   /* maximum allowed output value                        */
} pid_t;

/* Initialize PID controller with tuning parameters and output limits */
void pid_init(pid_t *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max);

/* Compute PID output given setpoint, current measurement and time delta in seconds */
float pid_compute(pid_t *pid, float setpoint, float measurement, float dt);

/* Reset integral and previous error — call when restarting the control loop */
void pid_reset(pid_t *pid);

#endif /* JGA25_370_PID_H */
