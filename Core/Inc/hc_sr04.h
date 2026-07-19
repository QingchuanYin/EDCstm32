#ifndef HC_SR04_H
#define HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef enum
{
    HC_SR04_IDLE = 0,
    HC_SR04_WAIT_RISING,
    HC_SR04_WAIT_FALLING
} HC_SR04_StateTypeDef;

typedef struct
{
    TIM_HandleTypeDef *timebase;
    GPIO_TypeDef *trigger_port;
    GPIO_TypeDef *echo_port;
    uint16_t trigger_pin;
    uint16_t echo_pin;
    volatile HC_SR04_StateTypeDef state;
    volatile uint16_t rise_count;
    volatile uint16_t pulse_width_us;
    volatile bool measurement_ready;
    volatile bool initialized;
    uint32_t trigger_tick_ms;
} HC_SR04_HandleTypeDef;

HAL_StatusTypeDef HC_SR04_Init(HC_SR04_HandleTypeDef *sensor,
                               TIM_HandleTypeDef *timebase,
                               GPIO_TypeDef *trigger_port,
                               uint16_t trigger_pin,
                               GPIO_TypeDef *echo_port,
                               uint16_t echo_pin);
HAL_StatusTypeDef HC_SR04_Trigger(HC_SR04_HandleTypeDef *sensor);
void HC_SR04_HandleEchoEdge(HC_SR04_HandleTypeDef *sensor);
bool HC_SR04_ReadPulseWidthUs(HC_SR04_HandleTypeDef *sensor, uint16_t *pulse_width_us);
bool HC_SR04_ReadDistanceCm(HC_SR04_HandleTypeDef *sensor, float *distance_cm);

#ifdef __cplusplus
}
#endif

#endif /* HC_SR04_H */
