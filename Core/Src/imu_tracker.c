#include "imu_tracker.h"

#include <math.h>
#include <string.h>

#define IMU_DEG_TO_RAD              0.01745329251994329577f
#define IMU_GRAVITY_MPS2            9.80665f
#define IMU_COMPLEMENTARY_ALPHA     0.98f
#define IMU_STATIONARY_ACCEL_BAND_G 0.05f
#define IMU_STATIONARY_GYRO_DPS     3.0f
#define IMU_STATIONARY_TIME_MS      200u
#define IMU_MAX_DT_MS               50u
#define IMU_NOMINAL_DT_MS           10u

static void IMU_Tracker_ClearMotion(IMU_Tracker_t *tracker, uint8_t clear_position)
{
    uint8_t axis;

    for (axis = 0u; axis < 3u; axis++)
    {
        tracker->velocity[axis] = 0.0f;
        tracker->previous_accel[axis] = 0.0f;
        if (clear_position != 0u)
        {
            tracker->position[axis] = 0.0f;
        }
    }
}

void IMU_Tracker_Init(IMU_Tracker_t *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    memset(tracker, 0, sizeof(*tracker));
}

void IMU_Tracker_ResetPosition(IMU_Tracker_t *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    IMU_Tracker_ClearMotion(tracker, 1u);
}

void IMU_Tracker_PrepareRecovery(IMU_Tracker_t *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    IMU_Tracker_ClearMotion(tracker, 0u);
    tracker->stationary_ms = 0u;
    tracker->initialized = 0u;
}

void IMU_Tracker_Update(IMU_Tracker_t *tracker, const MPU6050_t *sample, uint32_t dt_ms)
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float accel_roll;
    float accel_pitch;
    float dt;
    float accel_norm;
    float gyro_norm;
    float sr;
    float cr;
    float sp;
    float cp;
    float sy;
    float cy;
    float world_accel[3];
    uint8_t axis;

    if ((tracker == NULL) || (sample == NULL) || (dt_ms == 0u))
    {
        return;
    }

    ax = (float)sample->Ax;
    ay = (float)sample->Ay;
    az = (float)sample->Az;
    gx = (float)sample->Gx;
    gy = (float)sample->Gy;
    gz = (float)sample->Gz;

    accel_roll = atan2f(ay, az);
    accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (tracker->initialized == 0u)
    {
        tracker->roll = accel_roll;
        tracker->pitch = accel_pitch;
        tracker->yaw = 0.0f;
        tracker->initialized = 1u;
        return;
    }

    if (dt_ms > IMU_MAX_DT_MS)
    {
        dt_ms = IMU_NOMINAL_DT_MS;
        IMU_Tracker_ClearMotion(tracker, 0u);
    }
    dt = (float)dt_ms / 1000.0f;

    tracker->roll = IMU_COMPLEMENTARY_ALPHA *
                    (tracker->roll + gx * IMU_DEG_TO_RAD * dt) +
                    (1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_roll;
    tracker->pitch = IMU_COMPLEMENTARY_ALPHA *
                     (tracker->pitch + gy * IMU_DEG_TO_RAD * dt) +
                     (1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_pitch;
    tracker->yaw += gz * IMU_DEG_TO_RAD * dt;

    sr = sinf(tracker->roll);
    cr = cosf(tracker->roll);
    sp = sinf(tracker->pitch);
    cp = cosf(tracker->pitch);
    sy = sinf(tracker->yaw);
    cy = cosf(tracker->yaw);

    world_accel[0] = (cy * cp * ax + (cy * sp * sr - sy * cr) * ay +
                      (cy * sp * cr + sy * sr) * az) * IMU_GRAVITY_MPS2;
    world_accel[1] = (sy * cp * ax + (sy * sp * sr + cy * cr) * ay +
                      (sy * sp * cr - cy * sr) * az) * IMU_GRAVITY_MPS2;
    world_accel[2] = (-sp * ax + cp * sr * ay + cp * cr * az - 1.0f) *
                     IMU_GRAVITY_MPS2;

    accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    if ((fabsf(accel_norm - 1.0f) < IMU_STATIONARY_ACCEL_BAND_G) &&
        (gyro_norm < IMU_STATIONARY_GYRO_DPS))
    {
        if (tracker->stationary_ms < IMU_STATIONARY_TIME_MS)
        {
            tracker->stationary_ms += dt_ms;
        }
    }
    else
    {
        tracker->stationary_ms = 0u;
    }

    if (tracker->stationary_ms >= IMU_STATIONARY_TIME_MS)
    {
        IMU_Tracker_ClearMotion(tracker, 0u);
        return;
    }

    for (axis = 0u; axis < 3u; axis++)
    {
        float old_velocity = tracker->velocity[axis];
        float average_accel = 0.5f * (tracker->previous_accel[axis] + world_accel[axis]);

        tracker->velocity[axis] += average_accel * dt;
        tracker->position[axis] += 0.5f * (old_velocity + tracker->velocity[axis]) * dt;
        tracker->previous_accel[axis] = world_accel[axis];
    }
}
