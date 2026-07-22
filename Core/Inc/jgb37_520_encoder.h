#ifndef JGB37_520_ENCODER_H
#define JGB37_520_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define JGB37_520_ENCODER_BASE_PPR              11.0f
#define JGB37_520_ENCODER_QUADRATURE_FACTOR      4.0f
#define JGB37_520_GEAR_RATIO                     6.3f
#define JGB37_520_COUNTS_PER_OUTPUT_REV         \
    (JGB37_520_ENCODER_BASE_PPR *               \
     JGB37_520_ENCODER_QUADRATURE_FACTOR *      \
     JGB37_520_GEAR_RATIO)
#define JGB37_520_NOMINAL_OUTPUT_RPM          1580.0f

typedef struct
{
    int32_t sample_counts;
    int64_t total_counts;
    float counts_per_output_rev;
    float output_rpm;
} jgb37_520_encoder_t;

/* Initialize with the confirmed nominal value of 277.2 counts/output rev. */
bool jgb37_520_encoder_init(jgb37_520_encoder_t *encoder);

/* Initialize with a measured value to compensate for gearbox tolerances. */
bool jgb37_520_encoder_init_calibrated(
    jgb37_520_encoder_t *encoder,
    float counts_per_output_rev);

/* Update signed output-shaft speed from one sampling interval. */
bool jgb37_520_encoder_update(
    jgb37_520_encoder_t *encoder,
    int32_t delta_counts,
    float sample_period_s);

/* Calculate a signed delta from a wrapping 16-bit STM32 timer counter. */
int16_t jgb37_520_encoder_delta_u16(uint16_t current_count,
                                    uint16_t previous_count);

void jgb37_520_encoder_reset(jgb37_520_encoder_t *encoder);

#endif /* JGB37_520_ENCODER_H */
