/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "main.h"

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

typedef enum
{
    MPU6050_ERROR_NONE = 0,
    MPU6050_ERROR_NOT_FOUND,
    MPU6050_ERROR_BAD_ID,
    MPU6050_ERROR_CONFIG,
    MPU6050_ERROR_READ
} MPU6050_Error_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2Cx);

HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

HAL_StatusTypeDef MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

HAL_StatusTypeDef MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

MPU6050_Error_t MPU6050_GetLastError(void);
uint32_t MPU6050_GetLastI2CError(void);
uint8_t MPU6050_GetDeviceAddress(void);
uint8_t MPU6050_GetWhoAmI(void);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* MPU6050_H */
