/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* Board pin map: docs/broad.md */
#define TRACK_OUT_Pin                   GPIO_PIN_0
#define TRACK_OUT_GPIO_Port             GPIOA
#define TRACK_EN_Pin                    GPIO_PIN_1
#define TRACK_EN_GPIO_Port              GPIOA
#define TRACK_AD1_Pin                   GPIO_PIN_4
#define TRACK_AD1_GPIO_Port             GPIOA
#define TRACK_ERR_Pin                   GPIO_PIN_5
#define TRACK_ERR_GPIO_Port             GPIOA
#define TRACK_NC_Pin                    GPIO_PIN_6
#define TRACK_NC_GPIO_Port              GPIOA
#define KEY1_Pin                        GPIO_PIN_7
#define KEY1_GPIO_Port                  GPIOA
#define PWMB_Pin                        GPIO_PIN_8
#define PWMB_GPIO_Port                  GPIOA
#define PWMB_TIM_CHANNEL                TIM_CHANNEL_1
#define K230_USART1_TX_Pin              GPIO_PIN_9
#define K230_USART1_TX_GPIO_Port        GPIOA
#define K230_USART1_RX_Pin              GPIO_PIN_10
#define K230_USART1_RX_GPIO_Port        GPIOA
#define PWMA_Pin                        GPIO_PIN_11
#define PWMA_GPIO_Port                  GPIOA
#define PWMA_TIM_CHANNEL                TIM_CHANNEL_4
#define KEY2_Pin                        GPIO_PIN_12
#define KEY2_GPIO_Port                  GPIOA
#define MOTOR_B_ENCODER_CH1_Pin         GPIO_PIN_15
#define MOTOR_B_ENCODER_CH1_GPIO_Port   GPIOA

#define BIN2_Pin                        GPIO_PIN_0
#define BIN2_GPIO_Port                  GPIOB
#define BIN1_Pin                        GPIO_PIN_1
#define BIN1_GPIO_Port                  GPIOB
#define MOTOR_B_ENCODER_CH2_Pin         GPIO_PIN_3
#define MOTOR_B_ENCODER_CH2_GPIO_Port   GPIOB
#define TRACK_AD0_Pin                   GPIO_PIN_4
#define TRACK_AD0_GPIO_Port             GPIOB
#define TRACK_AD2_Pin                   GPIO_PIN_5
#define TRACK_AD2_GPIO_Port             GPIOB
#define MOTOR_A_ENCODER_CH1_Pin         GPIO_PIN_6
#define MOTOR_A_ENCODER_CH1_GPIO_Port   GPIOB
#define MOTOR_A_ENCODER_CH2_Pin         GPIO_PIN_7
#define MOTOR_A_ENCODER_CH2_GPIO_Port   GPIOB
#define MPU6050_SCL_Pin                 GPIO_PIN_8
#define MPU6050_SCL_GPIO_Port           GPIOB
#define MPU6050_SDA_Pin                 GPIO_PIN_9
#define MPU6050_SDA_GPIO_Port           GPIOB
#define SR04_ECHO_Pin                   GPIO_PIN_10
#define SR04_ECHO_GPIO_Port             GPIOB
#define SR04_TRIG_Pin                   GPIO_PIN_11
#define SR04_TRIG_GPIO_Port             GPIOB
#define AIN1_Pin                        GPIO_PIN_12
#define AIN1_GPIO_Port                  GPIOB
#define AIN2_Pin                        GPIO_PIN_13
#define AIN2_GPIO_Port                  GPIOB
#define BUZZER_Pin                      GPIO_PIN_14
#define BUZZER_GPIO_Port                GPIOB

#define OLED_SCL_Pin                    GPIO_PIN_14
#define OLED_SCL_GPIO_Port              GPIOC
#define OLED_SDA_Pin                    GPIO_PIN_15
#define OLED_SDA_GPIO_Port              GPIOC

extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
