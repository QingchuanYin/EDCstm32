#include "imu_filter.h"

#include <stddef.h>
#include <string.h>

#define IMU_FILTER_CALIBRATION_SAMPLES 100u
#define IMU_FILTER_ALPHA               0.20f
#define IMU_FILTER_ACCEL_NORM_MIN_SQ   0.64f
#define IMU_FILTER_ACCEL_NORM_MAX_SQ   1.44f
#define IMU_FILTER_GYRO_NORM_MAX_SQ    400.0f

void IMU_Filter_Init(IMU_Filter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    memset(filter, 0, sizeof(*filter));
}

uint8_t IMU_Filter_Update(IMU_Filter_t *filter, MPU6050_t *sample)
{
    float raw_accel[3];
    float raw_gyro[3];
    float accel_norm_squared;
    float gyro_norm_squared;
    uint8_t axis;

    if ((filter == NULL) || (sample == NULL))
    {
        return 0u;
    }

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

    if (filter->initialized == 0u)
    {
        for (axis = 0u; axis < 3u; axis++)
        {
            filter->accel[axis] = raw_accel[axis];
        }
        filter->initialized = 1u;
    }

    if (filter->calibrated == 0u)
    {
        if ((accel_norm_squared > IMU_FILTER_ACCEL_NORM_MIN_SQ) &&
            (accel_norm_squared < IMU_FILTER_ACCEL_NORM_MAX_SQ) &&
            (gyro_norm_squared < IMU_FILTER_GYRO_NORM_MAX_SQ))
        {
            filter->calibration_samples++;
            for (axis = 0u; axis < 3u; axis++)
            {
                filter->gyro_sum[axis] += raw_gyro[axis];
                filter->gyro_bias[axis] = filter->gyro_sum[axis] /
                                          (float)filter->calibration_samples;
            }
            if (filter->calibration_samples >= IMU_FILTER_CALIBRATION_SAMPLES)
            {
                filter->calibrated = 1u;
            }
        }
        else
        {
            for (axis = 0u; axis < 3u; axis++)
            {
                filter->gyro_sum[axis] = 0.0f;
                filter->gyro_bias[axis] = 0.0f;
            }
            filter->calibration_samples = 0u;
        }
    }

    for (axis = 0u; axis < 3u; axis++)
    {
        float corrected_gyro = raw_gyro[axis] - filter->gyro_bias[axis];

        filter->accel[axis] += IMU_FILTER_ALPHA *
                               (raw_accel[axis] - filter->accel[axis]);
        filter->gyro[axis] += IMU_FILTER_ALPHA *
                              (corrected_gyro - filter->gyro[axis]);
    }

    sample->Ax = filter->accel[0];
    sample->Ay = filter->accel[1];
    sample->Az = filter->accel[2];
    sample->Gx = filter->gyro[0];
    sample->Gy = filter->gyro[1];
    sample->Gz = filter->gyro[2];
    return filter->calibrated;
}

uint8_t IMU_Filter_IsCalibrated(const IMU_Filter_t *filter)
{
    return ((filter != NULL) && (filter->calibrated != 0u)) ? 1u : 0u;
}
