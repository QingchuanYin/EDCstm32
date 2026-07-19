#include "hc_sr04.h"

#define HC_SR04_TRIGGER_LOW_US     2u
#define HC_SR04_TRIGGER_HIGH_US    10u
#define HC_SR04_TIMEOUT_MS         40u
#define HC_SR04_CM_PER_US          0.01715f

static uint16_t HC_SR04_GetCounter(const HC_SR04_HandleTypeDef *sensor)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(sensor->timebase);
}

static void HC_SR04_DelayUs(const HC_SR04_HandleTypeDef *sensor, uint16_t delay_us)
{
    uint16_t start = HC_SR04_GetCounter(sensor);

    while ((uint16_t)(HC_SR04_GetCounter(sensor) - start) < delay_us)
    {
    }
}

HAL_StatusTypeDef HC_SR04_Init(HC_SR04_HandleTypeDef *sensor,
                               TIM_HandleTypeDef *timebase,
                               GPIO_TypeDef *trigger_port,
                               uint16_t trigger_pin,
                               GPIO_TypeDef *echo_port,
                               uint16_t echo_pin)
{
    if ((sensor == NULL) || (timebase == NULL) ||
        (trigger_port == NULL) || (echo_port == NULL))
    {
        return HAL_ERROR;
    }

    sensor->timebase = timebase;
    sensor->trigger_port = trigger_port;
    sensor->trigger_pin = trigger_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    sensor->state = HC_SR04_IDLE;
    sensor->rise_count = 0u;
    sensor->pulse_width_us = 0u;
    sensor->measurement_ready = false;
    sensor->initialized = false;
    sensor->trigger_tick_ms = 0u;

    HAL_GPIO_WritePin(sensor->trigger_port, sensor->trigger_pin, GPIO_PIN_RESET);
    if (HAL_TIM_Base_Start(sensor->timebase) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(sensor->echo_pin);
    sensor->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef HC_SR04_Trigger(HC_SR04_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || !sensor->initialized)
    {
        return HAL_ERROR;
    }

    if (sensor->state != HC_SR04_IDLE)
    {
        if ((HAL_GetTick() - sensor->trigger_tick_ms) <= HC_SR04_TIMEOUT_MS)
        {
            return HAL_BUSY;
        }
        sensor->state = HC_SR04_IDLE;
    }

    sensor->measurement_ready = false;
    sensor->trigger_tick_ms = HAL_GetTick();
    sensor->state = HC_SR04_WAIT_RISING;

    HAL_GPIO_WritePin(sensor->trigger_port, sensor->trigger_pin, GPIO_PIN_RESET);
    HC_SR04_DelayUs(sensor, HC_SR04_TRIGGER_LOW_US);
    HAL_GPIO_WritePin(sensor->trigger_port, sensor->trigger_pin, GPIO_PIN_SET);
    HC_SR04_DelayUs(sensor, HC_SR04_TRIGGER_HIGH_US);
    HAL_GPIO_WritePin(sensor->trigger_port, sensor->trigger_pin, GPIO_PIN_RESET);

    return HAL_OK;
}

void HC_SR04_HandleEchoEdge(HC_SR04_HandleTypeDef *sensor)
{
    uint16_t now;
    GPIO_PinState echo_state;

    if ((sensor == NULL) || !sensor->initialized)
    {
        return;
    }

    now = HC_SR04_GetCounter(sensor);
    echo_state = HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin);

    if ((echo_state == GPIO_PIN_SET) && (sensor->state == HC_SR04_WAIT_RISING))
    {
        sensor->rise_count = now;
        sensor->state = HC_SR04_WAIT_FALLING;
    }
    else if ((echo_state == GPIO_PIN_RESET) && (sensor->state == HC_SR04_WAIT_FALLING))
    {
        sensor->pulse_width_us = (uint16_t)(now - sensor->rise_count);
        sensor->measurement_ready = true;
        sensor->state = HC_SR04_IDLE;
    }
}

bool HC_SR04_ReadPulseWidthUs(HC_SR04_HandleTypeDef *sensor, uint16_t *pulse_width_us)
{
    if ((sensor == NULL) || (pulse_width_us == NULL) || !sensor->measurement_ready)
    {
        return false;
    }

    *pulse_width_us = sensor->pulse_width_us;
    sensor->measurement_ready = false;
    return true;
}

bool HC_SR04_ReadDistanceCm(HC_SR04_HandleTypeDef *sensor, float *distance_cm)
{
    uint16_t pulse_width_us;

    if ((distance_cm == NULL) || !HC_SR04_ReadPulseWidthUs(sensor, &pulse_width_us))
    {
        return false;
    }

    *distance_cm = (float)pulse_width_us * HC_SR04_CM_PER_US;
    return true;
}
