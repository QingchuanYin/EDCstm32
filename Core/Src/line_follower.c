#include "line_follower.h"

#include <stddef.h>

static float LineFollower_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void LineFollower_SetForwardDirection(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

static void LineFollower_SetDuty(LineFollower_HandleTypeDef *follower,
                                 float left_duty,
                                 float right_duty)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(follower->pwm_timer) + 1u;
    uint32_t left_compare;
    uint32_t right_compare;

    left_duty = LineFollower_Clamp(left_duty, 0.0f,
                                   LINE_FOLLOWER_MAX_DUTY_PERCENT);
    right_duty = LineFollower_Clamp(right_duty, 0.0f,
                                    LINE_FOLLOWER_MAX_DUTY_PERCENT);
    left_compare = (uint32_t)((left_duty * (float)period) / 100.0f);
    right_compare = (uint32_t)((right_duty * (float)period) / 100.0f);

    __HAL_TIM_SET_COMPARE(follower->pwm_timer, PWMA_TIM_CHANNEL, left_compare);
    __HAL_TIM_SET_COMPARE(follower->pwm_timer, PWMB_TIM_CHANNEL, right_compare);
    follower->left_duty_percent = (uint8_t)(left_duty + 0.5f);
    follower->right_duty_percent = (uint8_t)(right_duty + 0.5f);
}

static void LineFollower_MotorSafeStop(LineFollower_HandleTypeDef *follower)
{
    LineFollower_SetDuty(follower, 0.0f, 0.0f);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef LineFollower_Init(LineFollower_HandleTypeDef *follower,
                                    GraySensor_HandleTypeDef *sensor,
                                    TIM_HandleTypeDef *pwm_timer)
{
    if ((follower == NULL) || (sensor == NULL) || (pwm_timer == NULL))
    {
        return HAL_ERROR;
    }

    follower->sensor = sensor;
    follower->pwm_timer = pwm_timer;
    follower->last_update_ms = HAL_GetTick();
    follower->last_valid_ms = follower->last_update_ms;
    follower->previous_error = 0.0f;
    follower->last_correction = 0.0f;
    follower->left_duty_percent = 0u;
    follower->right_duty_percent = 0u;
    follower->state = LINE_FOLLOWER_STOPPED;
    follower->enabled = false;

    __HAL_TIM_SET_COMPARE(pwm_timer, PWMA_TIM_CHANNEL, 0u);
    __HAL_TIM_SET_COMPARE(pwm_timer, PWMB_TIM_CHANNEL, 0u);
    if (HAL_TIM_PWM_Start(pwm_timer, PWMA_TIM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(pwm_timer, PWMB_TIM_CHANNEL) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(pwm_timer, PWMA_TIM_CHANNEL);
        return HAL_ERROR;
    }

    LineFollower_MotorSafeStop(follower);
    return HAL_OK;
}

HAL_StatusTypeDef LineFollower_Start(LineFollower_HandleTypeDef *follower)
{
    if ((follower == NULL) || (follower->sensor == NULL) ||
        !follower->sensor->calibrated)
    {
        return HAL_ERROR;
    }

    follower->previous_error = 0.0f;
    follower->last_correction = 0.0f;
    follower->last_update_ms = HAL_GetTick() - LINE_FOLLOWER_PERIOD_MS;
    follower->last_valid_ms = HAL_GetTick();
    follower->enabled = true;
    follower->state = LINE_FOLLOWER_RUNNING;
    LineFollower_SetForwardDirection();
    LineFollower_SetDuty(follower, LINE_FOLLOWER_BASE_DUTY_PERCENT,
                         LINE_FOLLOWER_BASE_DUTY_PERCENT);
    return HAL_OK;
}

void LineFollower_Stop(LineFollower_HandleTypeDef *follower)
{
    if (follower == NULL)
    {
        return;
    }

    follower->enabled = false;
    follower->state = LINE_FOLLOWER_STOPPED;
    follower->previous_error = 0.0f;
    follower->last_correction = 0.0f;
    LineFollower_MotorSafeStop(follower);
}

HAL_StatusTypeDef LineFollower_Update(LineFollower_HandleTypeDef *follower,
                                      uint32_t now_ms)
{
    const GraySensor_ResultTypeDef *result;
    float correction;
    float error;

    if ((follower == NULL) || !follower->enabled)
    {
        return HAL_OK;
    }
    if ((now_ms - follower->last_update_ms) < LINE_FOLLOWER_PERIOD_MS)
    {
        return HAL_OK;
    }
    follower->last_update_ms = now_ms;

    if (GraySensor_Scan(follower->sensor) != HAL_OK)
    {
        follower->enabled = false;
        follower->state = LINE_FOLLOWER_FAULT;
        LineFollower_MotorSafeStop(follower);
        return HAL_ERROR;
    }

    result = GraySensor_GetResult(follower->sensor);
    if (result->line_valid)
    {
        error = (float)result->position / 1000.0f;
        correction = LINE_FOLLOWER_KP * error +
                     LINE_FOLLOWER_KD * (error - follower->previous_error);
        correction = LineFollower_Clamp(correction,
                                        -LINE_FOLLOWER_MAX_CORRECTION,
                                        LINE_FOLLOWER_MAX_CORRECTION);

        follower->previous_error = error;
        follower->last_correction = correction;
        follower->last_valid_ms = now_ms;
        follower->state = LINE_FOLLOWER_RUNNING;
        LineFollower_SetForwardDirection();
        LineFollower_SetDuty(follower,
                             LINE_FOLLOWER_BASE_DUTY_PERCENT + correction,
                             LINE_FOLLOWER_BASE_DUTY_PERCENT - correction);
    }
    else if ((now_ms - follower->last_valid_ms) <=
             LINE_FOLLOWER_LOST_HOLD_MS)
    {
        follower->state = LINE_FOLLOWER_LOST;
        LineFollower_SetForwardDirection();
        LineFollower_SetDuty(
            follower,
            LINE_FOLLOWER_BASE_DUTY_PERCENT + follower->last_correction,
            LINE_FOLLOWER_BASE_DUTY_PERCENT - follower->last_correction);
    }
    else
    {
        follower->state = LINE_FOLLOWER_LOST;
        LineFollower_MotorSafeStop(follower);
    }

    return HAL_OK;
}
