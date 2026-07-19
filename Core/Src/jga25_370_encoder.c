#include "jga25_370_encoder.h"

void encoder_init(encoder_t *enc, uint16_t ppr)
{
    enc->ppr         = ppr;
    enc->pulse_count = 0;
    enc->rpm         = 0.0f;
}

void encoder_update(encoder_t *enc, int32_t timer_count)
{
    enc->pulse_count = timer_count;
}

void encoder_compute_rpm(encoder_t *enc, float dt)
{
    /* RPM = (pulses / ppr) x (60 / dt)
     * pulses / ppr  — fractional revolutions this sample window
     * x (60 / dt)  — scale from per-second to per-minute                     */
    enc->rpm = ((float)enc->pulse_count / (float)enc->ppr) * (60.0f / dt);

    enc->pulse_count = 0;
}

void encoder_reset(encoder_t *enc)
{
    enc->pulse_count = 0;
    enc->rpm         = 0.0f;
}
