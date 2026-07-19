#ifndef JGA25_370_ENCODER_H
#define JGA25_370_ENCODER_H

#include <stdint.h>

/* Encoder state — holds pulse count and calculated RPM */
typedef struct
{
    int32_t  pulse_count;   /* raw pulse count captured from timer this cycle  */
    float    rpm;           /* output shaft speed in revolutions per minute     */
    uint16_t ppr;           /* effective pulses per revolution:
                             * encoder_ppr x quadrature_factor x gear_ratio    */
} encoder_t;

/* Initialize encoder with effective pulses per revolution.
 * ppr = physical_ppr x quadrature_factor x gear_ratio                        */
void encoder_init(encoder_t *enc, uint16_t ppr);

/* Store raw timer count captured since last reset */
void encoder_update(encoder_t *enc, int32_t timer_count);

/* Compute output shaft RPM from pulse count and sample period in seconds */
void encoder_compute_rpm(encoder_t *enc, float dt);

/* Reset pulse count and RPM — call when stopping the motor */
void encoder_reset(encoder_t *enc);

#endif /* JGA25_370_ENCODER_H */
