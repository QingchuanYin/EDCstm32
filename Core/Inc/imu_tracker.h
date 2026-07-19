#ifndef IMU_TRACKER_H
#define IMU_TRACKER_H

#include <stdint.h>

#include "mpu6050.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float velocity[3];
    float position[3];
    float previous_accel[3];
    uint32_t stationary_ms;
    uint8_t initialized;
} IMU_Tracker_t;

void IMU_Tracker_Init(IMU_Tracker_t *tracker);
void IMU_Tracker_Update(IMU_Tracker_t *tracker, const MPU6050_t *sample, uint32_t dt_ms);
void IMU_Tracker_ResetPosition(IMU_Tracker_t *tracker);
void IMU_Tracker_PrepareRecovery(IMU_Tracker_t *tracker);

#endif
