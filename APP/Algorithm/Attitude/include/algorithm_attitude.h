#ifndef ALGORITHM_ATTITUDE_H
#define ALGORITHM_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Algorithm_Attitude_Quaternion;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} Algorithm_Attitude_Euler;

typedef struct {
    /* Mahony nonlinear complementary-filter gains in 1/s and 1/s^2. */
    float proportional_gain;
    float integral_gain;
    float acceleration_reference_mps2;
    float acceleration_tolerance_mps2;
    float maximum_dt_seconds;
    float gyro_bias_rad_per_second[3];
} Algorithm_Attitude_Config;

typedef struct {
    float acceleration_mps2[3];
    float angular_rate_rad_per_second[3];
    float dt_seconds;
} Algorithm_Attitude_Input;

typedef struct {
    Algorithm_Attitude_Quaternion quaternion;
    Algorithm_Attitude_Euler euler;
    float gyro_bias_rad_per_second[3];
    uint32_t update_count;
    bool acceleration_correction_used;
    /* A 6-axis filter has no absolute heading observation. This remains false;
     * yaw is gyro integration and will drift with residual Z-axis bias. */
    bool yaw_absolute_observable;
    bool valid;
} Algorithm_Attitude_Snapshot;

typedef struct {
    Algorithm_Attitude_Config config;
    Algorithm_Attitude_Snapshot snapshot;
    float integral_feedback_rad_per_second[3];
    bool initialized;
} Algorithm_Attitude;

Algorithm_Attitude_Config Algorithm_Attitude_DefaultConfig(void);
bool Algorithm_Attitude_Init(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Config *config);

/* Input acceleration must use a body frame whose stationary level vector is
 * +Z. Gyroscope axes use the same right-handed body frame and radians/second.
 * Update is transactional: failure leaves context and output unchanged. */
bool Algorithm_Attitude_Update(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Input *input,
    Algorithm_Attitude_Snapshot *snapshot);

/* Quaternion input may be non-unit but must have a finite nonzero norm.
 * Setting orientation resets integral feedback but preserves configured bias. */
bool Algorithm_Attitude_SetQuaternion(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Quaternion *quaternion);
bool Algorithm_Attitude_SetGyroBias(
    Algorithm_Attitude *attitude,
    const float bias_rad_per_second[3]);
bool Algorithm_Attitude_GetSnapshot(
    const Algorithm_Attitude *attitude,
    Algorithm_Attitude_Snapshot *snapshot);

#endif
