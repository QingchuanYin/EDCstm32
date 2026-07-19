#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>

#include "mpu6050.h"

typedef struct
{
    float accel[3];
    float gyro[3];
    float gyro_sum[3];
    float gyro_bias[3];
    uint16_t calibration_samples;
    uint8_t initialized;
    uint8_t calibrated;
} IMU_Filter_t;

void IMU_Filter_Init(IMU_Filter_t *filter);
uint8_t IMU_Filter_Update(IMU_Filter_t *filter, MPU6050_t *sample);
uint8_t IMU_Filter_IsCalibrated(const IMU_Filter_t *filter);

#endif
