/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

static const uint16_t i2c_timeout = 20;
static uint16_t mpu6050_address = (0x68u << 1);
static uint8_t mpu6050_who_am_i;
static MPU6050_Error_t mpu6050_last_error;
static uint32_t mpu6050_last_i2c_error;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static uint8_t MPU6050_IsSupportedId(uint8_t id)
{
    /* Some modules sold as MPU6050 use an MPU6500-compatible die (WHO_AM_I=0x70). */
    return ((id == 0x68u) || (id == 0x70u)) ? 1u : 0u;
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    static const uint16_t addresses[] = {(0x68u << 1), (0x69u << 1)};
    uint8_t check = 0u;
    uint8_t Data;
    uint8_t index;
    uint8_t device_responded = 0u;
    HAL_StatusTypeDef status;

    if (I2Cx == NULL)
    {
        return HAL_ERROR;
    }

    mpu6050_last_error = MPU6050_ERROR_NOT_FOUND;
    mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
    mpu6050_address = 0u;
    mpu6050_who_am_i = 0u;

    for (index = 0u; index < (uint8_t)(sizeof(addresses) / sizeof(addresses[0])); index++)
    {
        check = 0u;
        status = HAL_I2C_Mem_Read(I2Cx, addresses[index], WHO_AM_I_REG,
                                  I2C_MEMADD_SIZE_8BIT, &check, 1, i2c_timeout);
        if (status == HAL_OK)
        {
            device_responded = 1u;
            mpu6050_address = addresses[index];
            mpu6050_who_am_i = check;
            mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
            if (MPU6050_IsSupportedId(check) != 0u)
            {
                break;
            }
        }
        else
        {
            mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        }
    }

    if (MPU6050_IsSupportedId(check) == 0u)
    {
        mpu6050_last_error = (device_responded != 0u) ? MPU6050_ERROR_BAD_ID :
                                                      MPU6050_ERROR_NOT_FOUND;
        return HAL_ERROR;
    }

    /* Wake the device and use the X gyro PLL as the clock source. */
    Data = 0x01u;
    status = HAL_I2C_Mem_Write(I2Cx, mpu6050_address, PWR_MGMT_1_REG,
                               I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_CONFIG;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    /* DLPF_CFG=3 gives about 44 Hz accelerometer/gyro bandwidth. */
    Data = 0x03u;
    status = HAL_I2C_Mem_Write(I2Cx, mpu6050_address, CONFIG_REG,
                               I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_CONFIG;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    /* With DLPF enabled, 1 kHz / (1 + 9) gives a 100 Hz sample rate. */
    Data = 0x09u;
    status = HAL_I2C_Mem_Write(I2Cx, mpu6050_address, SMPLRT_DIV_REG,
                               I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_CONFIG;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    /* Accelerometer +/-2 g and gyro +/-250 degrees/s. */
    Data = 0x00u;
    status = HAL_I2C_Mem_Write(I2Cx, mpu6050_address, ACCEL_CONFIG_REG,
                               I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_CONFIG;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }
    status = HAL_I2C_Mem_Write(I2Cx, mpu6050_address, GYRO_CONFIG_REG,
                               I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status == HAL_OK)
    {
        timer = HAL_GetTick();
        mpu6050_last_error = MPU6050_ERROR_NONE;
        mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
    }
    else
    {
        mpu6050_last_error = MPU6050_ERROR_CONFIG;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
    }
    return status;
}

HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    HAL_StatusTypeDef status;

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    status = HAL_I2C_Mem_Read(I2Cx, mpu6050_address, ACCEL_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_READ;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    HAL_StatusTypeDef status;

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    status = HAL_I2C_Mem_Read(I2Cx, mpu6050_address, GYRO_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_READ;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;
    HAL_StatusTypeDef status;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    status = HAL_I2C_Mem_Read(I2Cx, mpu6050_address, TEMP_OUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT, Rec_Data, 2, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_READ;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;
    HAL_StatusTypeDef status;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    status = HAL_I2C_Mem_Read(I2Cx, mpu6050_address, ACCEL_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT, Rec_Data, 14, i2c_timeout);
    if (status != HAL_OK)
    {
        mpu6050_last_error = MPU6050_ERROR_READ;
        mpu6050_last_i2c_error = HAL_I2C_GetError(I2Cx);
        return status;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        (double)DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW +
        (double)DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
    {
        DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, -DataStruct->Gx, dt);
    }
    else
    {
        DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    }
    mpu6050_last_error = MPU6050_ERROR_NONE;
    mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
    return HAL_OK;
}

MPU6050_Error_t MPU6050_GetLastError(void)
{
    return mpu6050_last_error;
}

uint32_t MPU6050_GetLastI2CError(void)
{
    return mpu6050_last_i2c_error;
}

uint8_t MPU6050_GetDeviceAddress(void)
{
    return (uint8_t)(mpu6050_address >> 1);
}

uint8_t MPU6050_GetWhoAmI(void)
{
    return mpu6050_who_am_i;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
