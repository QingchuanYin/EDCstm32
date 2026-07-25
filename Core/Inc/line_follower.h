#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "gray_sensor.h"
#include "main.h"

#define LINE_FOLLOWER_PERIOD_MS          10u
#define LINE_FOLLOWER_BASE_DUTY_PERCENT   5.0f
#define LINE_FOLLOWER_MAX_DUTY_PERCENT    5.0f
#define LINE_FOLLOWER_KP                 20.0f
#define LINE_FOLLOWER_KD                 60.0f
#define LINE_FOLLOWER_MAX_CORRECTION      5.0f
#define LINE_FOLLOWER_LOST_HOLD_MS       100u

typedef enum
{
    LINE_FOLLOWER_STOPPED = 0,
    LINE_FOLLOWER_RUNNING,
    LINE_FOLLOWER_LOST,
    LINE_FOLLOWER_FAULT
} LineFollower_StateTypeDef;

typedef struct
{
    GraySensor_HandleTypeDef *sensor;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t last_update_ms;
    uint32_t last_valid_ms;
    float previous_error;
    float last_correction;
    uint8_t left_duty_percent;
    uint8_t right_duty_percent;
    LineFollower_StateTypeDef state;
    bool enabled;
} LineFollower_HandleTypeDef;

HAL_StatusTypeDef LineFollower_Init(LineFollower_HandleTypeDef *follower,
                                    GraySensor_HandleTypeDef *sensor,
                                    TIM_HandleTypeDef *pwm_timer);
HAL_StatusTypeDef LineFollower_Start(LineFollower_HandleTypeDef *follower);
void LineFollower_Stop(LineFollower_HandleTypeDef *follower);
HAL_StatusTypeDef LineFollower_Update(LineFollower_HandleTypeDef *follower,
                                      uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWER_H */
