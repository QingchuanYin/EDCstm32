#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define GRAY_SENSOR_CHANNEL_COUNT       8u
#define GRAY_SENSOR_STRENGTH_MAX        1000u
#define GRAY_SENSOR_MIN_CAL_SPAN        200u
#define GRAY_SENSOR_NOISE_FLOOR         150u
#define GRAY_SENSOR_LINE_MIN_SUM        350u
#define GRAY_SENSOR_LINE_MIN_PEAK       350u
#define GRAY_SENSOR_LINE_MIN_CONTRAST   250u

typedef struct
{
    uint16_t raw[GRAY_SENSOR_CHANNEL_COUNT];
    uint16_t strength[GRAY_SENSOR_CHANNEL_COUNT];
    int16_t position;
    uint16_t confidence;
    uint16_t peak;
    uint16_t contrast;
    GPIO_PinState err_state;
    bool line_valid;
} GraySensor_ResultTypeDef;

typedef struct
{
    ADC_HandleTypeDef *adc;
    TIM_HandleTypeDef *timebase;
    uint16_t white[GRAY_SENSOR_CHANNEL_COUNT];
    uint16_t black[GRAY_SENSOR_CHANNEL_COUNT];
    GraySensor_ResultTypeDef result;
    bool white_captured;
    bool calibrating;
    bool calibrated;
} GraySensor_HandleTypeDef;

HAL_StatusTypeDef GraySensor_Init(GraySensor_HandleTypeDef *sensor,
                                  ADC_HandleTypeDef *adc,
                                  TIM_HandleTypeDef *timebase);
HAL_StatusTypeDef GraySensor_Scan(GraySensor_HandleTypeDef *sensor);
HAL_StatusTypeDef GraySensor_CaptureWhite(GraySensor_HandleTypeDef *sensor,
                                          uint16_t sample_count);
HAL_StatusTypeDef GraySensor_BeginBlackCalibration(GraySensor_HandleTypeDef *sensor);
HAL_StatusTypeDef GraySensor_UpdateBlackCalibration(GraySensor_HandleTypeDef *sensor);
bool GraySensor_FinishBlackCalibration(GraySensor_HandleTypeDef *sensor,
                                       uint8_t *failed_channels);
const GraySensor_ResultTypeDef *GraySensor_GetResult(
    const GraySensor_HandleTypeDef *sensor);

#ifdef __cplusplus
}
#endif

#endif /* GRAY_SENSOR_H */
