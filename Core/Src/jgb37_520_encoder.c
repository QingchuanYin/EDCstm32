#include "jgb37_520_encoder.h"

#include <stddef.h>

/*
 * JGB37-520 parameterization informed by:
 * https://github.com/chenzhaoqiJM/stm32-JGB37-520-control
 * Reimplemented for this STM32 HAL project; no upstream source was copied.
 */

bool jgb37_520_encoder_init(jgb37_520_encoder_t *encoder)
{
    return jgb37_520_encoder_init_calibrated(
        encoder, JGB37_520_COUNTS_PER_OUTPUT_REV);
}

bool jgb37_520_encoder_init_calibrated(
    jgb37_520_encoder_t *encoder,
    float counts_per_output_rev)
{
    if ((encoder == NULL) || (counts_per_output_rev <= 0.0f))
    {
        return false;
    }

    encoder->counts_per_output_rev = counts_per_output_rev;
    encoder->sample_counts = 0;
    encoder->total_counts = 0;
    encoder->output_rpm = 0.0f;
    return true;
}

bool jgb37_520_encoder_update(
    jgb37_520_encoder_t *encoder,
    int32_t delta_counts,
    float sample_period_s)
{
    if ((encoder == NULL) ||
        (encoder->counts_per_output_rev <= 0.0f) ||
        (sample_period_s <= 0.0f))
    {
        return false;
    }

    encoder->sample_counts = delta_counts;
    encoder->total_counts += delta_counts;
    encoder->output_rpm =
        ((float)delta_counts * 60.0f) /
        (encoder->counts_per_output_rev * sample_period_s);
    return true;
}

int16_t jgb37_520_encoder_delta_u16(uint16_t current_count,
                                    uint16_t previous_count)
{
    return (int16_t)(uint16_t)(current_count - previous_count);
}

void jgb37_520_encoder_reset(jgb37_520_encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->sample_counts = 0;
    encoder->total_counts = 0;
    encoder->output_rpm = 0.0f;
}
