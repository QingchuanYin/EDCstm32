#include "gray_sensor.h"

#include <stddef.h>

#define GRAY_SENSOR_SETTLE_US           10u
#define GRAY_SENSOR_ADC_TIMEOUT_MS       2u

static const int16_t gray_sensor_weights[GRAY_SENSOR_CHANNEL_COUNT] =
{
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

static uint16_t GraySensor_GetCounter(const GraySensor_HandleTypeDef *sensor)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(sensor->timebase);
}

static void GraySensor_DelayUs(const GraySensor_HandleTypeDef *sensor,
                               uint16_t delay_us)
{
    uint16_t start = GraySensor_GetCounter(sensor);

    while ((uint16_t)(GraySensor_GetCounter(sensor) - start) < delay_us)
    {
    }
}

static uint16_t GraySensor_AbsDiff(uint16_t first, uint16_t second)
{
    return (first >= second) ? (uint16_t)(first - second)
                             : (uint16_t)(second - first);
}

static void GraySensor_SelectChannel(uint8_t channel)
{
    HAL_GPIO_WritePin(TRACK_AD0_GPIO_Port, TRACK_AD0_Pin,
                      ((channel & 0x01u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_AD1_GPIO_Port, TRACK_AD1_Pin,
                      ((channel & 0x02u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_AD2_GPIO_Port, TRACK_AD2_Pin,
                      ((channel & 0x04u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static HAL_StatusTypeDef GraySensor_ReadChannel(GraySensor_HandleTypeDef *sensor,
                                                uint8_t channel,
                                                uint16_t *value)
{
    HAL_StatusTypeDef status;

    GraySensor_SelectChannel(channel);
    GraySensor_DelayUs(sensor, GRAY_SENSOR_SETTLE_US);

    status = HAL_ADC_Start(sensor->adc);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_PollForConversion(sensor->adc,
                                       GRAY_SENSOR_ADC_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        *value = (uint16_t)HAL_ADC_GetValue(sensor->adc);
    }
    (void)HAL_ADC_Stop(sensor->adc);
    return status;
}

static void GraySensor_Process(GraySensor_HandleTypeDef *sensor)
{
    int32_t weighted_sum = 0;
    uint32_t strength_sum = 0u;
    uint16_t maximum_strength = 0u;
    uint16_t minimum_strength = GRAY_SENSOR_STRENGTH_MAX;
    uint8_t channel;

    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        int32_t delta = (int32_t)sensor->result.raw[channel] -
                        (int32_t)sensor->white[channel];
        int32_t span = (int32_t)sensor->black[channel] -
                       (int32_t)sensor->white[channel];
        int32_t strength = 0;

        if (sensor->calibrated && (span != 0))
        {
            strength = (delta * (int32_t)GRAY_SENSOR_STRENGTH_MAX) / span;
            if (strength < 0)
            {
                strength = 0;
            }
            else if (strength > (int32_t)GRAY_SENSOR_STRENGTH_MAX)
            {
                strength = (int32_t)GRAY_SENSOR_STRENGTH_MAX;
            }

            if (strength <= (int32_t)GRAY_SENSOR_NOISE_FLOOR)
            {
                strength = 0;
            }
            else
            {
                strength =
                    ((strength - (int32_t)GRAY_SENSOR_NOISE_FLOOR) *
                     (int32_t)GRAY_SENSOR_STRENGTH_MAX) /
                    ((int32_t)GRAY_SENSOR_STRENGTH_MAX -
                     (int32_t)GRAY_SENSOR_NOISE_FLOOR);
            }
        }

        sensor->result.strength[channel] = (uint16_t)strength;
        if ((uint16_t)strength > maximum_strength)
        {
            maximum_strength = (uint16_t)strength;
        }
        if ((uint16_t)strength < minimum_strength)
        {
            minimum_strength = (uint16_t)strength;
        }
        strength_sum += (uint32_t)strength;
        weighted_sum += strength * gray_sensor_weights[channel];
    }

    sensor->result.confidence =
        (strength_sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)strength_sum;
    sensor->result.peak = maximum_strength;
    sensor->result.contrast =
        (uint16_t)(maximum_strength - minimum_strength);
    sensor->result.line_valid = sensor->calibrated &&
        (strength_sum >= GRAY_SENSOR_LINE_MIN_SUM) &&
        (maximum_strength >= GRAY_SENSOR_LINE_MIN_PEAK) &&
        (sensor->result.contrast >=
         GRAY_SENSOR_LINE_MIN_CONTRAST);

    if (sensor->result.line_valid)
    {
        int32_t position = weighted_sum / (int32_t)strength_sum;
        position = (position * 1000) / 3500;
        if (position < -1000)
        {
            position = -1000;
        }
        else if (position > 1000)
        {
            position = 1000;
        }
        sensor->result.position = (int16_t)position;
    }
}

HAL_StatusTypeDef GraySensor_Init(GraySensor_HandleTypeDef *sensor,
                                  ADC_HandleTypeDef *adc,
                                  TIM_HandleTypeDef *timebase)
{
    uint8_t channel;

    if ((sensor == NULL) || (adc == NULL) || (timebase == NULL))
    {
        return HAL_ERROR;
    }

    sensor->adc = adc;
    sensor->timebase = timebase;
    sensor->white_captured = false;
    sensor->calibrating = false;
    sensor->calibrated = false;
    sensor->result.position = 0;
    sensor->result.confidence = 0u;
    sensor->result.peak = 0u;
    sensor->result.contrast = 0u;
    sensor->result.err_state = GPIO_PIN_RESET;
    sensor->result.line_valid = false;

    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        sensor->white[channel] = 0u;
        sensor->black[channel] = 0u;
        sensor->result.raw[channel] = 0u;
        sensor->result.strength[channel] = 0u;
    }

    GraySensor_SelectChannel(0u);
    HAL_GPIO_WritePin(TRACK_EN_GPIO_Port, TRACK_EN_Pin, GPIO_PIN_SET);

    if (HAL_ADCEx_Calibration_Start(sensor->adc) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef GraySensor_Scan(GraySensor_HandleTypeDef *sensor)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t channel;

    if ((sensor == NULL) || (sensor->adc == NULL) ||
        (sensor->timebase == NULL))
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(TRACK_EN_GPIO_Port, TRACK_EN_Pin, GPIO_PIN_RESET);
    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        status = GraySensor_ReadChannel(sensor, channel,
                                        &sensor->result.raw[channel]);
        if (status != HAL_OK)
        {
            break;
        }
    }
    HAL_GPIO_WritePin(TRACK_EN_GPIO_Port, TRACK_EN_Pin, GPIO_PIN_SET);

    sensor->result.err_state =
        HAL_GPIO_ReadPin(TRACK_ERR_GPIO_Port, TRACK_ERR_Pin);
    if (status == HAL_OK)
    {
        GraySensor_Process(sensor);
    }
    else
    {
        sensor->result.line_valid = false;
    }
    return status;
}

HAL_StatusTypeDef GraySensor_CaptureWhite(GraySensor_HandleTypeDef *sensor,
                                          uint16_t sample_count)
{
    uint32_t sums[GRAY_SENSOR_CHANNEL_COUNT] = {0u};
    uint16_t sample;
    uint8_t channel;

    if ((sensor == NULL) || (sample_count == 0u))
    {
        return HAL_ERROR;
    }

    sensor->calibrated = false;
    sensor->calibrating = false;
    for (sample = 0u; sample < sample_count; sample++)
    {
        if (GraySensor_Scan(sensor) != HAL_OK)
        {
            sensor->white_captured = false;
            return HAL_ERROR;
        }
        for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
        {
            sums[channel] += sensor->result.raw[channel];
        }
    }

    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        sensor->white[channel] = (uint16_t)(sums[channel] / sample_count);
        sensor->black[channel] = sensor->white[channel];
    }
    sensor->white_captured = true;
    return HAL_OK;
}

HAL_StatusTypeDef GraySensor_BeginBlackCalibration(GraySensor_HandleTypeDef *sensor)
{
    uint8_t channel;

    if ((sensor == NULL) || !sensor->white_captured)
    {
        return HAL_ERROR;
    }

    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        sensor->black[channel] = sensor->white[channel];
    }
    sensor->calibrated = false;
    sensor->calibrating = true;
    return HAL_OK;
}

HAL_StatusTypeDef GraySensor_UpdateBlackCalibration(GraySensor_HandleTypeDef *sensor)
{
    uint8_t channel;

    if ((sensor == NULL) || !sensor->calibrating)
    {
        return HAL_ERROR;
    }
    if (GraySensor_Scan(sensor) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        if (GraySensor_AbsDiff(sensor->result.raw[channel],
                               sensor->white[channel]) >
            GraySensor_AbsDiff(sensor->black[channel],
                               sensor->white[channel]))
        {
            sensor->black[channel] = sensor->result.raw[channel];
        }
    }
    return HAL_OK;
}

bool GraySensor_FinishBlackCalibration(GraySensor_HandleTypeDef *sensor,
                                       uint8_t *failed_channels)
{
    uint8_t failed = 0u;
    uint8_t channel;

    if ((sensor == NULL) || !sensor->calibrating)
    {
        return false;
    }

    sensor->calibrating = false;
    for (channel = 0u; channel < GRAY_SENSOR_CHANNEL_COUNT; channel++)
    {
        if (GraySensor_AbsDiff(sensor->black[channel],
                               sensor->white[channel]) <
            GRAY_SENSOR_MIN_CAL_SPAN)
        {
            failed |= (uint8_t)(1u << channel);
        }
    }

    sensor->calibrated = (failed == 0u);
    sensor->result.line_valid = false;
    if (failed_channels != NULL)
    {
        *failed_channels = failed;
    }
    return sensor->calibrated;
}

const GraySensor_ResultTypeDef *GraySensor_GetResult(
    const GraySensor_HandleTypeDef *sensor)
{
    return (sensor == NULL) ? NULL : &sensor->result;
}
