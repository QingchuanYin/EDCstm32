/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "hc_sr04.h"
#include "gray_sensor.h"
#include "line_follower.h"
#include "oled.h"
#include "serial_console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_WAIT_WHITE = 0,
  APP_WAIT_SWEEP,
  APP_SWEEPING,
  APP_READY,
  APP_RUNNING,
  APP_CAL_FAILED,
  APP_FAULT
} AppStateTypeDef;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState active_state;
  GPIO_PinState raw_state;
  GPIO_PinState stable_state;
  uint32_t changed_at_ms;
} ButtonTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_MS              30u
#define WHITE_CALIBRATION_SAMPLES       32u
#define BLACK_CALIBRATION_MS          5000u
#define READY_SCAN_PERIOD_MS            50u
#define OLED_REFRESH_MS                250u
#define SERIAL_OLED_TEXT_MAX            16u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HC_SR04_HandleTypeDef hsr04;
GraySensor_HandleTypeDef gray_sensor;
LineFollower_HandleTypeDef line_follower;
ButtonTypeDef key1_button;
ButtonTypeDef key2_button;
AppStateTypeDef app_state = APP_WAIT_WHITE;
uint32_t calibration_started_ms;
uint32_t last_calibration_scan_ms;
uint32_t last_ready_scan_ms;
uint32_t last_oled_refresh_ms;
uint8_t calibration_failed_channels;
char serial_oled_text[SERIAL_OLED_TEXT_MAX + 1u];
uint8_t serial_oled_dirty = 1u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
/* USER CODE BEGIN PFP */
static void App_SetState(AppStateTypeDef state);
static void App_UpdateDisplay(uint32_t now_ms);
static void App_GetSerialStatus(SerialConsole_StatusTypeDef *status);
static bool App_SetSerialDisplay(const char *text);
static void Button_Init(ButtonTypeDef *button,
                        GPIO_TypeDef *port,
                        uint16_t pin,
                        GPIO_PinState active_state);
static bool Button_Pressed(ButtonTypeDef *button, uint32_t now_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Button_Init(ButtonTypeDef *button,
                        GPIO_TypeDef *port,
                        uint16_t pin,
                        GPIO_PinState active_state)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);

  button->port = port;
  button->pin = pin;
  button->active_state = active_state;
  button->raw_state = state;
  button->stable_state = state;
  button->changed_at_ms = HAL_GetTick();
}

static bool Button_Pressed(ButtonTypeDef *button, uint32_t now_ms)
{
  GPIO_PinState raw = HAL_GPIO_ReadPin(button->port, button->pin);

  if (raw != button->raw_state)
  {
    button->raw_state = raw;
    button->changed_at_ms = now_ms;
  }
  else if ((raw != button->stable_state) &&
           ((now_ms - button->changed_at_ms) >= BUTTON_DEBOUNCE_MS))
  {
    button->stable_state = raw;
    return raw == button->active_state;
  }
  return false;
}

static void App_SetState(AppStateTypeDef state)
{
  if ((state != APP_RUNNING) && line_follower.enabled)
  {
    LineFollower_Stop(&line_follower);
  }
  app_state = state;
  last_oled_refresh_ms = 0u;
}

static const char *App_StateText(void)
{
  if ((app_state == APP_RUNNING) &&
      (line_follower.state == LINE_FOLLOWER_LOST))
  {
    return "LOST ";
  }

  switch (app_state)
  {
    case APP_WAIT_WHITE: return "WHITE";
    case APP_WAIT_SWEEP: return "SWEEP";
    case APP_SWEEPING:   return "CAL  ";
    case APP_READY:      return "READY";
    case APP_RUNNING:    return "RUN  ";
    case APP_CAL_FAILED: return "FAIL ";
    default:             return "FAULT";
  }
}

static const char *App_StateProtocolText(void)
{
  if ((app_state == APP_RUNNING) &&
      (line_follower.state == LINE_FOLLOWER_LOST))
  {
    return "LOST";
  }

  switch (app_state)
  {
    case APP_WAIT_WHITE: return "WHITE";
    case APP_WAIT_SWEEP: return "SWEEP";
    case APP_SWEEPING:   return "CAL";
    case APP_READY:      return "READY";
    case APP_RUNNING:    return "RUN";
    case APP_CAL_FAILED: return "FAIL";
    default:             return "FAULT";
  }
}

static void App_GetSerialStatus(SerialConsole_StatusTypeDef *status)
{
  const GraySensor_ResultTypeDef *result =
      GraySensor_GetResult(&gray_sensor);

  status->state = App_StateProtocolText();
  status->uptime_ms = HAL_GetTick();
  status->calibrated = gray_sensor.calibrated;
  status->line_valid = result->line_valid;
  status->position = result->position;
  status->confidence = result->confidence;
  status->left_duty_percent = line_follower.left_duty_percent;
  status->right_duty_percent = line_follower.right_duty_percent;
}

static bool App_SetSerialDisplay(const char *text)
{
  size_t length;
  size_t i;

  if (text == NULL)
  {
    return false;
  }
  length = strlen(text);
  if ((length == 0u) || (length > SERIAL_OLED_TEXT_MAX))
  {
    return false;
  }
  for (i = 0u; i < length; i++)
  {
    if (((unsigned char)text[i] < 0x20u) ||
        ((unsigned char)text[i] > 0x7eu))
    {
      return false;
    }
  }

  memcpy(serial_oled_text, text, length + 1u);
  serial_oled_dirty = 1u;
  last_oled_refresh_ms = 0u;
  return true;
}

static void App_UpdateDisplay(uint32_t now_ms)
{
  const GraySensor_ResultTypeDef *result;
  static AppStateTypeDef displayed_state = APP_FAULT;
  bool state_changed;
  uint32_t remaining_s;

  if ((displayed_state == app_state) &&
      ((now_ms - last_oled_refresh_ms) < OLED_REFRESH_MS))
  {
    return;
  }
  last_oled_refresh_ms = now_ms;
  result = GraySensor_GetResult(&gray_sensor);
  state_changed = displayed_state != app_state;

  if (state_changed)
  {
    OLED_Clear();
    OLED_ShowString(0u, 0u, "8CH TRACK");
    OLED_ShowString(0u, 1u, "State:");
    displayed_state = app_state;
  }
  OLED_ShowString(42u, 1u, App_StateText());

  if (app_state == APP_WAIT_WHITE)
  {
    OLED_ShowString(0u, 3u, "On white ground");
    OLED_ShowString(0u, 5u, "K1: capture white");
  }
  else if (app_state == APP_WAIT_SWEEP)
  {
    OLED_ShowString(0u, 3u, "Move across line");
    OLED_ShowString(0u, 5u, "K1: start 5 sec");
  }
  else if (app_state == APP_SWEEPING)
  {
    remaining_s = BLACK_CALIBRATION_MS -
                  (now_ms - calibration_started_ms);
    remaining_s = (remaining_s + 999u) / 1000u;
    OLED_ShowString(0u, 3u, "Sweep all 8 CH");
    OLED_ShowString(0u, 5u, "Remain:");
    OLED_ShowNum(48u, 5u, remaining_s, 1u);
    OLED_ShowString(54u, 5u, "s ");
  }
  else if (app_state == APP_CAL_FAILED)
  {
    OLED_ShowString(0u, 3u, "Bad CH mask:");
    OLED_ShowNum(78u, 3u, calibration_failed_channels, 3u);
    OLED_ShowString(0u, 5u, "K1: retry white");
  }
  else if ((app_state == APP_READY) || (app_state == APP_RUNNING))
  {
    OLED_ShowString(0u, 2u, "Line:");
    OLED_ShowString(30u, 2u, result->line_valid ? "YES" : "NO ");
    OLED_ShowString(0u, 3u, "Pos:");
    OLED_ShowSignedNum(30u, 3u, result->position, 4u);
    OLED_ShowString(72u, 3u, "C:");
    OLED_ShowNum(84u, 3u, result->confidence, 4u);
    OLED_ShowString(0u, 4u, "P:");
    OLED_ShowNum(12u, 4u, result->peak, 4u);
    OLED_ShowString(48u, 4u, "D:");
    OLED_ShowNum(60u, 4u, result->contrast, 4u);
    OLED_ShowString(0u, 5u, "L:");
    OLED_ShowNum(12u, 5u, line_follower.left_duty_percent, 2u);
    OLED_ShowString(30u, 5u, "R:");
    OLED_ShowNum(42u, 5u, line_follower.right_duty_percent, 2u);
    OLED_ShowString(66u, 5u, "ERR:");
    OLED_ShowNum(90u, 5u,
                 (result->err_state == GPIO_PIN_SET) ? 1u : 0u, 1u);
    OLED_ShowString(0u, 7u,
                    (app_state == APP_READY) ? "K2: start" : "K2: stop ");
  }
  else
  {
    OLED_ShowString(0u, 3u, "ADC/control error");
    OLED_ShowString(0u, 5u, "Reset required");
  }
  if (state_changed || (serial_oled_dirty != 0u))
  {
    OLED_ShowString(0u, 6u, "UART:");
    OLED_ShowString(30u, 6u, "                ");
    OLED_ShowString(30u, 6u, serial_oled_text);
    serial_oled_dirty = 0u;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HC_SR04_Init(&hsr04, &htim3,
                   SR04_TRIG_GPIO_Port, SR04_TRIG_Pin,
                   SR04_ECHO_GPIO_Port, SR04_ECHO_Pin) != HAL_OK)
  {
    Error_Handler();
  }

  OLED_Init();
  OLED_Clear();
  if (GraySensor_Init(&gray_sensor, &hadc1, &htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (LineFollower_Init(&line_follower, &gray_sensor, &htim1) != HAL_OK)
  {
    Error_Handler();
  }
  Button_Init(&key1_button, KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_SET);
  Button_Init(&key2_button, KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET);
  App_SetState(APP_WAIT_WHITE);
  if (SerialConsole_Init(&huart1,
                         App_GetSerialStatus,
                         App_SetSerialDisplay) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    SerialConsole_Process(now_ms);

    if (Button_Pressed(&key1_button, now_ms))
    {
      if ((app_state == APP_WAIT_WHITE) ||
          (app_state == APP_READY) ||
          (app_state == APP_CAL_FAILED))
      {
        LineFollower_Stop(&line_follower);
        if (GraySensor_CaptureWhite(&gray_sensor,
                                    WHITE_CALIBRATION_SAMPLES) == HAL_OK)
        {
          App_SetState(APP_WAIT_SWEEP);
        }
        else
        {
          App_SetState(APP_FAULT);
        }
      }
      else if (app_state == APP_WAIT_SWEEP)
      {
        if (GraySensor_BeginBlackCalibration(&gray_sensor) == HAL_OK)
        {
          calibration_started_ms = now_ms;
          last_calibration_scan_ms = now_ms - LINE_FOLLOWER_PERIOD_MS;
          App_SetState(APP_SWEEPING);
        }
        else
        {
          App_SetState(APP_FAULT);
        }
      }
    }

    if (app_state == APP_SWEEPING)
    {
      if ((now_ms - last_calibration_scan_ms) >= LINE_FOLLOWER_PERIOD_MS)
      {
        last_calibration_scan_ms = now_ms;
        if (GraySensor_UpdateBlackCalibration(&gray_sensor) != HAL_OK)
        {
          App_SetState(APP_FAULT);
        }
      }
      if ((app_state == APP_SWEEPING) &&
          ((now_ms - calibration_started_ms) >= BLACK_CALIBRATION_MS))
      {
        if (GraySensor_FinishBlackCalibration(
              &gray_sensor, &calibration_failed_channels))
        {
          App_SetState(APP_READY);
        }
        else
        {
          App_SetState(APP_CAL_FAILED);
        }
      }
    }

    if (Button_Pressed(&key2_button, now_ms))
    {
      if (app_state == APP_READY)
      {
        if (LineFollower_Start(&line_follower) == HAL_OK)
        {
          App_SetState(APP_RUNNING);
        }
      }
      else if (app_state == APP_RUNNING)
      {
        LineFollower_Stop(&line_follower);
        App_SetState(APP_READY);
      }
    }

    if ((app_state == APP_RUNNING) &&
        (LineFollower_Update(&line_follower, now_ms) != HAL_OK))
    {
      App_SetState(APP_FAULT);
    }

    if ((app_state == APP_READY) &&
        ((now_ms - last_ready_scan_ms) >= READY_SCAN_PERIOD_MS))
    {
      last_ready_scan_ms = now_ms;
      if (GraySensor_Scan(&gray_sensor) != HAL_OK)
      {
        App_SetState(APP_FAULT);
      }
    }
    App_UpdateDisplay(now_ms);
  }
  /* USER CODE END 3 */
}

/**
  * @brief ADC1 Initialization Function
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 PWM Initialization Function
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWMB_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWMA_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Encoder Initialization Function
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 1 MHz time base Initialization Function
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Encoder Initialization Function
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_SET);

  /* Motor, ultrasonic and buzzer outputs default to their inactive state. */
  HAL_GPIO_WritePin(GPIOB, BIN2_Pin|BIN1_Pin|SR04_TRIG_Pin|AIN2_Pin
                           |AIN1_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /* Keep the analog multiplexer disabled until a scan starts. */
  HAL_GPIO_WritePin(TRACK_EN_GPIO_Port, TRACK_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TRACK_AD1_GPIO_Port, TRACK_AD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TRACK_AD0_Pin|TRACK_AD2_Pin, GPIO_PIN_RESET);

  /* OLED software I2C */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);

  /* Motor direction, HC-SR04 trigger and buzzer outputs */
  GPIO_InitStruct.Pin = BIN2_Pin|BIN1_Pin|SR04_TRIG_Pin|AIN2_Pin
                        |AIN1_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* HC-SR04 echo edges are timestamped with the 1 MHz TIM3 time base. */
  GPIO_InitStruct.Pin = SR04_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SR04_ECHO_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* 74HC4051 enable and address outputs. */
  GPIO_InitStruct.Pin = TRACK_EN_Pin|TRACK_AD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TRACK_AD0_Pin|TRACK_AD2_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ERR is diagnostic only; PA6 is deliberately left in analog mode. */
  GPIO_InitStruct.Pin = TRACK_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRACK_ERR_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TRACK_NC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(TRACK_NC_GPIO_Port, &GPIO_InitStruct);

  /* KEY1 is active high; KEY2 is active low. */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SR04_ECHO_Pin)
  {
    HC_SR04_HandleEchoEdge(&hsr04);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
