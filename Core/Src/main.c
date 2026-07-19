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
#include "hc_sr04.h"
#include "imu_tracker.h"
#include "mpu6050.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float accel[3];
  float gyro[3];
  float gyro_sum[3];
  float gyro_bias[3];
  uint16_t calibration_samples;
  uint8_t initialized;
  uint8_t calibrated;
} App_IMUFilter_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_SAMPLE_INTERVAL_MS  10u
#define OLED_REFRESH_INTERVAL_MS 50u
#define MPU_RETRY_INTERVAL_MS   500u
#define KEY_DEBOUNCE_MS         20u
#define IMU_CALIBRATION_SAMPLES 100u
#define IMU_FILTER_ALPHA        0.20f
#define IMU_DISPLAY_ACCEL_STEP  0.01f
#define IMU_DISPLAY_ACCEL_HYST  0.015f
#define IMU_DISPLAY_GYRO_STEP   0.1f
#define IMU_DISPLAY_GYRO_HYST   0.3f
#define IMU_DISPLAY_GYRO_ZERO   0.3f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HC_SR04_HandleTypeDef hsr04;
MPU6050_t mpu6050;
IMU_Tracker_t imu_tracker;
static uint8_t mpu_ready;
static uint8_t key_candidate_pressed;
static uint8_t key_stable_pressed;
static uint32_t key_change_tick;
static uint32_t last_sample_tick;
static uint32_t last_display_tick;
static uint32_t last_retry_tick;
static uint32_t key_feedback_tick;
static uint8_t key_feedback_active;
static App_IMUFilter_t imu_filter;
static float displayed_accel[3];
static float displayed_gyro[3];
static uint8_t displayed_imu_initialized;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
/* USER CODE BEGIN PFP */
static void App_HandleKey(uint32_t now);
static void App_ShowMeasurements(void);
static void App_ShowMpuError(uint32_t now);
static uint8_t App_TryInitMpu(uint32_t now);
static void App_RecoverI2cBus(void);
static void App_ResetImuFilter(void);
static uint8_t App_FilterMpuSample(MPU6050_t *sample);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t App_Pow10(uint8_t exponent)
{
  uint32_t result = 1u;

  while (exponent > 0u)
  {
    result *= 10u;
    exponent--;
  }
  return result;
}

static float App_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float App_QuantizeFloat(float value, float step)
{
  float scaled = value / step;
  int32_t rounded = (scaled < 0.0f) ? (int32_t)(scaled - 0.5f) :
                                      (int32_t)(scaled + 0.5f);

  return (float)rounded * step;
}

static void App_ResetImuFilter(void)
{
  uint8_t axis;

  for (axis = 0u; axis < 3u; axis++)
  {
    imu_filter.accel[axis] = 0.0f;
    imu_filter.gyro[axis] = 0.0f;
    imu_filter.gyro_sum[axis] = 0.0f;
    imu_filter.gyro_bias[axis] = 0.0f;
  }
  imu_filter.calibration_samples = 0u;
  imu_filter.initialized = 0u;
  imu_filter.calibrated = 0u;
  displayed_imu_initialized = 0u;
}

static uint8_t App_FilterMpuSample(MPU6050_t *sample)
{
  float raw_accel[3];
  float raw_gyro[3];
  float accel_norm_squared;
  float gyro_norm_squared;
  uint8_t axis;

  raw_accel[0] = (float)sample->Ax;
  raw_accel[1] = (float)sample->Ay;
  raw_accel[2] = (float)sample->Az;
  raw_gyro[0] = (float)sample->Gx;
  raw_gyro[1] = (float)sample->Gy;
  raw_gyro[2] = (float)sample->Gz;
  accel_norm_squared = raw_accel[0] * raw_accel[0] +
                       raw_accel[1] * raw_accel[1] +
                       raw_accel[2] * raw_accel[2];
  gyro_norm_squared = raw_gyro[0] * raw_gyro[0] +
                      raw_gyro[1] * raw_gyro[1] +
                      raw_gyro[2] * raw_gyro[2];

  if (imu_filter.initialized == 0u)
  {
    for (axis = 0u; axis < 3u; axis++)
    {
      imu_filter.accel[axis] = raw_accel[axis];
      imu_filter.gyro[axis] = 0.0f;
    }
    imu_filter.initialized = 1u;
  }

  if (imu_filter.calibrated == 0u)
  {
    /* Only accept calibration samples while the module is approximately still. */
    if ((accel_norm_squared > 0.64f) && (accel_norm_squared < 1.44f) &&
        (gyro_norm_squared < 400.0f))
    {
      imu_filter.calibration_samples++;
      for (axis = 0u; axis < 3u; axis++)
      {
        imu_filter.gyro_sum[axis] += raw_gyro[axis];
        imu_filter.gyro_bias[axis] = imu_filter.gyro_sum[axis] /
                                       (float)imu_filter.calibration_samples;
      }
      if (imu_filter.calibration_samples >= IMU_CALIBRATION_SAMPLES)
      {
        imu_filter.calibrated = 1u;
        displayed_imu_initialized = 0u;
        IMU_Tracker_PrepareRecovery(&imu_tracker);
      }
    }
    else
    {
      for (axis = 0u; axis < 3u; axis++)
      {
        imu_filter.gyro_sum[axis] = 0.0f;
        imu_filter.gyro_bias[axis] = 0.0f;
      }
      imu_filter.calibration_samples = 0u;
    }
  }

  for (axis = 0u; axis < 3u; axis++)
  {
    float corrected_gyro = raw_gyro[axis] - imu_filter.gyro_bias[axis];

    imu_filter.accel[axis] += IMU_FILTER_ALPHA *
                              (raw_accel[axis] - imu_filter.accel[axis]);
    imu_filter.gyro[axis] += IMU_FILTER_ALPHA *
                             (corrected_gyro - imu_filter.gyro[axis]);
  }

  sample->Ax = imu_filter.accel[0];
  sample->Ay = imu_filter.accel[1];
  sample->Az = imu_filter.accel[2];
  sample->Gx = imu_filter.gyro[0];
  sample->Gy = imu_filter.gyro[1];
  sample->Gz = imu_filter.gyro[2];
  return imu_filter.calibrated;
}

static void App_UpdateDisplayedImu(void)
{
  const float accel[3] = {(float)mpu6050.Ax, (float)mpu6050.Ay, (float)mpu6050.Az};
  const float gyro[3] = {(float)mpu6050.Gx, (float)mpu6050.Gy, (float)mpu6050.Gz};
  uint8_t axis;

  if (displayed_imu_initialized == 0u)
  {
    for (axis = 0u; axis < 3u; axis++)
    {
      displayed_accel[axis] = App_QuantizeFloat(accel[axis], IMU_DISPLAY_ACCEL_STEP);
      displayed_gyro[axis] = (App_AbsFloat(gyro[axis]) < IMU_DISPLAY_GYRO_ZERO) ? 0.0f :
                             App_QuantizeFloat(gyro[axis], IMU_DISPLAY_GYRO_STEP);
    }
    displayed_imu_initialized = 1u;
    return;
  }

  for (axis = 0u; axis < 3u; axis++)
  {
    float gyro_target = (App_AbsFloat(gyro[axis]) < IMU_DISPLAY_GYRO_ZERO) ? 0.0f :
                        gyro[axis];

    if (App_AbsFloat(accel[axis] - displayed_accel[axis]) >= IMU_DISPLAY_ACCEL_HYST)
    {
      displayed_accel[axis] = App_QuantizeFloat(accel[axis], IMU_DISPLAY_ACCEL_STEP);
    }
    if (App_AbsFloat(gyro_target - displayed_gyro[axis]) >= IMU_DISPLAY_GYRO_HYST)
    {
      displayed_gyro[axis] = App_QuantizeFloat(gyro_target, IMU_DISPLAY_GYRO_STEP);
    }
  }
}

static void App_FormatAxisValue(char *line, char type, char axis, float value,
                                uint8_t whole_digits, uint8_t fraction_digits,
                                const char *unit)
{
  uint8_t digit;
  uint8_t index = 0u;
  uint8_t total_digits = (uint8_t)(whole_digits + fraction_digits);
  uint32_t scale = App_Pow10(fraction_digits);
  uint32_t limit = App_Pow10(total_digits) - 1u;
  uint32_t divisor = App_Pow10((uint8_t)(total_digits - 1u));
  uint32_t scaled;
  float magnitude = value;

  line[index++] = type;
  line[index++] = axis;
  line[index++] = ':';
  if (magnitude < 0.0f)
  {
    line[index++] = '-';
    magnitude = -magnitude;
  }
  else
  {
    line[index++] = '+';
  }

  if (magnitude >= ((float)limit / (float)scale))
  {
    scaled = limit;
  }
  else
  {
    scaled = (uint32_t)(magnitude * (float)scale + 0.5f);
  }

  for (digit = 0u; digit < total_digits; digit++)
  {
    if (digit == whole_digits)
    {
      line[index++] = '.';
    }
    line[index++] = (char)('0' + (scaled / divisor) % 10u);
    divisor /= 10u;
  }

  while (*unit != '\0')
  {
    line[index++] = *unit++;
  }
  line[index] = '\0';
}

static void App_DrawAxisLine(uint8_t row, char type, char axis, float value,
                             uint8_t whole_digits, uint8_t fraction_digits,
                             const char *unit)
{
  char line[22];

  App_FormatAxisValue(line, type, axis, value, whole_digits, fraction_digits, unit);
  OLED_BufferShowString(0u, (uint8_t)(row * 7u), line);
}

static void App_ShowMeasurements(void)
{
  App_UpdateDisplayedImu();
  OLED_BufferClear();
  App_DrawAxisLine(0u, 'A', 'X', displayed_accel[0], 1u, 2u, "g");
  App_DrawAxisLine(1u, 'A', 'Y', displayed_accel[1], 1u, 2u, "g");
  App_DrawAxisLine(2u, 'A', 'Z', displayed_accel[2], 1u, 2u, "g");
  App_DrawAxisLine(3u, 'G', 'X', displayed_gyro[0], 3u, 1u, "d/s");
  App_DrawAxisLine(4u, 'G', 'Y', displayed_gyro[1], 3u, 1u, "d/s");
  App_DrawAxisLine(5u, 'G', 'Z', displayed_gyro[2], 3u, 1u, "d/s");
  App_DrawAxisLine(6u, 'P', 'X', imu_tracker.position[0], 3u, 3u, "m");
  App_DrawAxisLine(7u, 'P', 'Y', imu_tracker.position[1], 3u, 3u, "m");
  App_DrawAxisLine(8u, 'P', 'Z', imu_tracker.position[2], 3u, 3u, "m");
  OLED_BufferFlush();
}

static void App_FormatHex(char *line, const char *prefix, uint32_t value, uint8_t digits)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t index = 0u;
  uint8_t digit;

  while (*prefix != '\0')
  {
    line[index++] = *prefix++;
  }
  for (digit = 0u; digit < digits; digit++)
  {
    uint8_t shift = (uint8_t)((digits - digit - 1u) * 4u);
    line[index++] = hex[(value >> shift) & 0x0Fu];
  }
  line[index] = '\0';
}

static const char *App_GetMpuErrorText(void)
{
  switch (MPU6050_GetLastError())
  {
    case MPU6050_ERROR_BAD_ID:
      return "STAGE:BAD ID";
    case MPU6050_ERROR_CONFIG:
      return "STAGE:CONFIG";
    case MPU6050_ERROR_READ:
      return "STAGE:READ";
    case MPU6050_ERROR_NOT_FOUND:
    default:
      return "STAGE:NO DEVICE";
  }
}

static void App_ShowMpuError(uint32_t now)
{
  char line[22];

  OLED_BufferClear();
  OLED_BufferShowString(0u, 0u, "MPU6050 ERROR");
  OLED_BufferShowString(0u, 14u, App_GetMpuErrorText());
  App_FormatHex(line, "I2C:", MPU6050_GetLastI2CError(), 8u);
  OLED_BufferShowString(0u, 21u, line);
  App_FormatHex(line, "ADDR:", MPU6050_GetDeviceAddress(), 2u);
  OLED_BufferShowString(0u, 28u, line);
  App_FormatHex(line, "ID:", MPU6050_GetWhoAmI(), 2u);
  OLED_BufferShowString(60u, 28u, line);
  OLED_BufferShowString(0u, 42u, "RETRY 68/69 100K");
  if ((key_feedback_active != 0u) &&
      ((uint32_t)(now - key_feedback_tick) < 500u))
  {
    OLED_BufferShowString(0u, 56u, "KEY1 DETECTED");
  }
  else
  {
    key_feedback_active = 0u;
    OLED_BufferShowString(0u, 56u, "CHECK PB8/PB9");
  }
  OLED_BufferFlush();
}

static void App_HandleKey(uint32_t now)
{
  uint8_t pressed = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) ? 1u : 0u;

  if (pressed != key_candidate_pressed)
  {
    key_candidate_pressed = pressed;
    key_change_tick = now;
  }
  else if ((pressed != key_stable_pressed) &&
           ((uint32_t)(now - key_change_tick) >= KEY_DEBOUNCE_MS))
  {
    key_stable_pressed = pressed;
    if (key_stable_pressed != 0u)
    {
      IMU_Tracker_ResetPosition(&imu_tracker);
      key_feedback_tick = now;
      key_feedback_active = 1u;
      last_display_tick = now - OLED_REFRESH_INTERVAL_MS;
    }
  }
}

static void App_I2cRecoveryDelay(void)
{
  volatile uint32_t delay;

  for (delay = 0u; delay < 64u; delay++)
  {
    __NOP();
  }
}

static void App_RecoverI2cBus(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t pulse;

  __HAL_I2C_DISABLE(&hi2c1);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  GPIO_InitStruct.Pin = MPU6050_SCL_Pin | MPU6050_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin | MPU6050_SDA_Pin, GPIO_PIN_SET);
  App_I2cRecoveryDelay();
  for (pulse = 0u; pulse < 9u; pulse++)
  {
    HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_RESET);
    App_I2cRecoveryDelay();
    HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_SET);
    App_I2cRecoveryDelay();
  }

  HAL_GPIO_WritePin(GPIOB, MPU6050_SDA_Pin, GPIO_PIN_RESET);
  App_I2cRecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_SET);
  App_I2cRecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, MPU6050_SDA_Pin, GPIO_PIN_SET);
  App_I2cRecoveryDelay();

  hi2c1.State = HAL_I2C_STATE_RESET;
  MX_I2C1_Init();
}

static uint8_t App_TryInitMpu(uint32_t now)
{
  if (MPU6050_Init(&hi2c1) != HAL_OK)
  {
    App_RecoverI2cBus();
    if (MPU6050_Init(&hi2c1) != HAL_OK)
    {
      return 0u;
    }
  }
  if (MPU6050_Read_All(&hi2c1, &mpu6050) != HAL_OK)
  {
    return 0u;
  }

  if (App_FilterMpuSample(&mpu6050) != 0u)
  {
    IMU_Tracker_Update(&imu_tracker, &mpu6050, MPU_SAMPLE_INTERVAL_MS);
  }
  last_sample_tick = now;
  return 1u;
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
  IMU_Tracker_Init(&imu_tracker);
  App_ResetImuFilter();

  key_candidate_pressed =
      (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) ? 1u : 0u;
  key_stable_pressed = key_candidate_pressed;
  key_change_tick = HAL_GetTick();
  last_sample_tick = HAL_GetTick();
  last_display_tick = last_sample_tick - OLED_REFRESH_INTERVAL_MS;
  last_retry_tick = last_sample_tick;
  mpu_ready = App_TryInitMpu(last_sample_tick);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    App_HandleKey(now);

    if (mpu_ready != 0u)
    {
      uint32_t elapsed = (uint32_t)(now - last_sample_tick);

      if (elapsed >= MPU_SAMPLE_INTERVAL_MS)
      {
        last_sample_tick = now;
        if (MPU6050_Read_All(&hi2c1, &mpu6050) == HAL_OK)
        {
          if (App_FilterMpuSample(&mpu6050) != 0u)
          {
            IMU_Tracker_Update(&imu_tracker, &mpu6050, elapsed);
          }
        }
        else
        {
          mpu_ready = 0u;
          last_retry_tick = now;
          IMU_Tracker_PrepareRecovery(&imu_tracker);
          App_ResetImuFilter();
        }
      }
    }
    else if ((uint32_t)(now - last_retry_tick) >= MPU_RETRY_INTERVAL_MS)
    {
      last_retry_tick = now;
      if (App_TryInitMpu(now) != 0u)
      {
        mpu_ready = 1u;
      }
    }

    if ((uint32_t)(now - last_display_tick) >= OLED_REFRESH_INTERVAL_MS)
    {
      last_display_tick = now;
      if (mpu_ready != 0u)
      {
        App_ShowMeasurements();
      }
      else
      {
        App_ShowMpuError(now);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

  /* Line tracking sensor digital inputs */
  GPIO_InitStruct.Pin = L3_Pin|L2_Pin|TRACK_M_Pin|R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = R1_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
