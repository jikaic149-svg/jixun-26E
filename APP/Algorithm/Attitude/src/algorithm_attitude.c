#include "algorithm_attitude.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define ALGORITHM_ATTITUDE_PI 3.14159265358979323846F
#define ALGORITHM_ATTITUDE_RAD_TO_DEG \
    (180.0F / ALGORITHM_ATTITUDE_PI)

static bool finite3(const float values[3])
{
    return (values != NULL) && isfinite(values[0]) &&
        isfinite(values[1]) && isfinite(values[2]);
}

static bool normalize_quaternion(Algorithm_Attitude_Quaternion *quaternion)
{
    float norm_squared;
    float inverse_norm;

    if ((quaternion == NULL) || !isfinite(quaternion->w) ||
        !isfinite(quaternion->x) || !isfinite(quaternion->y) ||
        !isfinite(quaternion->z)) {
        return false;
    }
    norm_squared = (quaternion->w * quaternion->w) +
        (quaternion->x * quaternion->x) +
        (quaternion->y * quaternion->y) +
        (quaternion->z * quaternion->z);
    if (!isfinite(norm_squared) || (norm_squared <= 0.0F)) {
        return false;
    }
    inverse_norm = 1.0F / sqrtf(norm_squared);
    if (!isfinite(inverse_norm)) {
        return false;
    }
    quaternion->w *= inverse_norm;
    quaternion->x *= inverse_norm;
    quaternion->y *= inverse_norm;
    quaternion->z *= inverse_norm;
    return isfinite(quaternion->w) && isfinite(quaternion->x) &&
        isfinite(quaternion->y) && isfinite(quaternion->z);
}

static bool quaternion_to_euler(
    const Algorithm_Attitude_Quaternion *quaternion,
    Algorithm_Attitude_Euler *euler)
{
    float pitch_sine;
    float roll_numerator;
    float roll_denominator;
    float yaw_numerator;
    float yaw_denominator;

    if ((quaternion == NULL) || (euler == NULL)) {
        return false;
    }
    roll_numerator = 2.0F *
        ((quaternion->w * quaternion->x) +
         (quaternion->y * quaternion->z));
    roll_denominator = 1.0F - (2.0F *
        ((quaternion->x * quaternion->x) +
         (quaternion->y * quaternion->y)));
    pitch_sine = 2.0F *
        ((quaternion->w * quaternion->y) -
         (quaternion->z * quaternion->x));
    if (pitch_sine > 1.0F) {
        pitch_sine = 1.0F;
    }
    else if (pitch_sine < -1.0F) {
        pitch_sine = -1.0F;
    }
    yaw_numerator = 2.0F *
        ((quaternion->w * quaternion->z) +
         (quaternion->x * quaternion->y));
    yaw_denominator = 1.0F - (2.0F *
        ((quaternion->y * quaternion->y) +
         (quaternion->z * quaternion->z)));

    euler->roll_deg = atan2f(roll_numerator, roll_denominator) *
        ALGORITHM_ATTITUDE_RAD_TO_DEG;
    euler->pitch_deg = asinf(pitch_sine) *
        ALGORITHM_ATTITUDE_RAD_TO_DEG;
    euler->yaw_deg = atan2f(yaw_numerator, yaw_denominator) *
        ALGORITHM_ATTITUDE_RAD_TO_DEG;
    return isfinite(euler->roll_deg) && isfinite(euler->pitch_deg) &&
        isfinite(euler->yaw_deg);
}

static uint32_t increment_saturating(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : value + 1U;
}

static bool valid_config(const Algorithm_Attitude_Config *config)
{
    return (config != NULL) && isfinite(config->proportional_gain) &&
        isfinite(config->integral_gain) &&
        isfinite(config->acceleration_reference_mps2) &&
        isfinite(config->acceleration_tolerance_mps2) &&
        isfinite(config->maximum_dt_seconds) &&
        finite3(config->gyro_bias_rad_per_second) &&
        (config->proportional_gain >= 0.0F) &&
        (config->integral_gain >= 0.0F) &&
        (config->acceleration_reference_mps2 > 0.0F) &&
        (config->acceleration_tolerance_mps2 >= 0.0F) &&
        (config->maximum_dt_seconds > 0.0F);
}

Algorithm_Attitude_Config Algorithm_Attitude_DefaultConfig(void)
{
    const Algorithm_Attitude_Config config = {
        .proportional_gain = 2.0F,
        .integral_gain = 0.05F,
        .acceleration_reference_mps2 = 9.80665F,
        .acceleration_tolerance_mps2 = 3.0F,
        .maximum_dt_seconds = 0.050F,
        .gyro_bias_rad_per_second = {0.0F, 0.0F, 0.0F},
    };
    return config;
}

bool Algorithm_Attitude_Init(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Config *config)
{
    const Algorithm_Attitude_Config selected = (config == NULL)
        ? Algorithm_Attitude_DefaultConfig()
        : *config;

    if (attitude != NULL) {
        *attitude = (Algorithm_Attitude){0};
    }
    if ((attitude == NULL) || !valid_config(&selected)) {
        return false;
    }
    attitude->config = selected;
    attitude->snapshot.quaternion.w = 1.0F;
    attitude->snapshot.gyro_bias_rad_per_second[0] =
        selected.gyro_bias_rad_per_second[0];
    attitude->snapshot.gyro_bias_rad_per_second[1] =
        selected.gyro_bias_rad_per_second[1];
    attitude->snapshot.gyro_bias_rad_per_second[2] =
        selected.gyro_bias_rad_per_second[2];
    attitude->snapshot.yaw_absolute_observable = false;
    attitude->snapshot.valid = true;
    attitude->initialized = true;
    return true;
}

bool Algorithm_Attitude_Update(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Input *input,
    Algorithm_Attitude_Snapshot *snapshot)
{
    Algorithm_Attitude next;
    Algorithm_Attitude_Quaternion quaternion;
    float gyro[3];
    float acceleration_norm_squared;
    float acceleration_norm;
    float normalized_acceleration[3];
    float estimated_gravity[3];
    float error[3];
    float half_dt;
    float previous_w;
    float previous_x;
    float previous_y;
    float previous_z;
    bool correction_used = false;
    unsigned int axis;

    if ((attitude == NULL) || !attitude->initialized || (input == NULL) ||
        (snapshot == NULL) || !finite3(input->acceleration_mps2) ||
        !finite3(input->angular_rate_rad_per_second) ||
        !isfinite(input->dt_seconds) || (input->dt_seconds <= 0.0F) ||
        (input->dt_seconds > attitude->config.maximum_dt_seconds)) {
        return false;
    }

    next = *attitude;
    quaternion = next.snapshot.quaternion;
    for (axis = 0U; axis < 3U; ++axis) {
        gyro[axis] = input->angular_rate_rad_per_second[axis] -
            next.config.gyro_bias_rad_per_second[axis];
        if (!isfinite(gyro[axis])) {
            return false;
        }
    }

    acceleration_norm_squared =
        (input->acceleration_mps2[0] * input->acceleration_mps2[0]) +
        (input->acceleration_mps2[1] * input->acceleration_mps2[1]) +
        (input->acceleration_mps2[2] * input->acceleration_mps2[2]);
    if (!isfinite(acceleration_norm_squared)) {
        return false;
    }
    if (acceleration_norm_squared > 0.0F) {
        acceleration_norm = sqrtf(acceleration_norm_squared);
        if (!isfinite(acceleration_norm)) {
            return false;
        }
        correction_used =
            fabsf(acceleration_norm -
                next.config.acceleration_reference_mps2) <=
            next.config.acceleration_tolerance_mps2;
    }

    if (correction_used) {
        const float inverse_acceleration_norm = 1.0F / acceleration_norm;

        for (axis = 0U; axis < 3U; ++axis) {
            normalized_acceleration[axis] =
                input->acceleration_mps2[axis] *
                inverse_acceleration_norm;
        }
        estimated_gravity[0] = 2.0F *
            ((quaternion.x * quaternion.z) -
             (quaternion.w * quaternion.y));
        estimated_gravity[1] = 2.0F *
            ((quaternion.w * quaternion.x) +
             (quaternion.y * quaternion.z));
        estimated_gravity[2] =
            (quaternion.w * quaternion.w) -
            (quaternion.x * quaternion.x) -
            (quaternion.y * quaternion.y) +
            (quaternion.z * quaternion.z);
        error[0] =
            (normalized_acceleration[1] * estimated_gravity[2]) -
            (normalized_acceleration[2] * estimated_gravity[1]);
        error[1] =
            (normalized_acceleration[2] * estimated_gravity[0]) -
            (normalized_acceleration[0] * estimated_gravity[2]);
        error[2] =
            (normalized_acceleration[0] * estimated_gravity[1]) -
            (normalized_acceleration[1] * estimated_gravity[0]);

        for (axis = 0U; axis < 3U; ++axis) {
            next.integral_feedback_rad_per_second[axis] +=
                next.config.integral_gain * error[axis] *
                input->dt_seconds;
            gyro[axis] +=
                (next.config.proportional_gain * error[axis]) +
                next.integral_feedback_rad_per_second[axis];
            if (!isfinite(next.integral_feedback_rad_per_second[axis]) ||
                !isfinite(gyro[axis])) {
                return false;
            }
        }
    }

    half_dt = 0.5F * input->dt_seconds;
    previous_w = quaternion.w;
    previous_x = quaternion.x;
    previous_y = quaternion.y;
    previous_z = quaternion.z;
    quaternion.w +=
        (-previous_x * gyro[0] - previous_y * gyro[1] -
         previous_z * gyro[2]) * half_dt;
    quaternion.x +=
        (previous_w * gyro[0] + previous_y * gyro[2] -
         previous_z * gyro[1]) * half_dt;
    quaternion.y +=
        (previous_w * gyro[1] - previous_x * gyro[2] +
         previous_z * gyro[0]) * half_dt;
    quaternion.z +=
        (previous_w * gyro[2] + previous_x * gyro[1] -
         previous_y * gyro[0]) * half_dt;
    if (!normalize_quaternion(&quaternion) ||
        !quaternion_to_euler(&quaternion, &next.snapshot.euler)) {
        return false;
    }

    next.snapshot.quaternion = quaternion;
    next.snapshot.update_count =
        increment_saturating(next.snapshot.update_count);
    next.snapshot.acceleration_correction_used = correction_used;
    next.snapshot.yaw_absolute_observable = false;
    next.snapshot.valid = true;
    *attitude = next;
    *snapshot = next.snapshot;
    return true;
}

bool Algorithm_Attitude_SetQuaternion(
    Algorithm_Attitude *attitude,
    const Algorithm_Attitude_Quaternion *quaternion)
{
    Algorithm_Attitude_Quaternion normalized;
    Algorithm_Attitude_Euler euler;

    if ((attitude == NULL) || !attitude->initialized ||
        (quaternion == NULL)) {
        return false;
    }
    normalized = *quaternion;
    if (!normalize_quaternion(&normalized) ||
        !quaternion_to_euler(&normalized, &euler)) {
        return false;
    }
    attitude->snapshot.quaternion = normalized;
    attitude->snapshot.euler = euler;
    attitude->integral_feedback_rad_per_second[0] = 0.0F;
    attitude->integral_feedback_rad_per_second[1] = 0.0F;
    attitude->integral_feedback_rad_per_second[2] = 0.0F;
    attitude->snapshot.acceleration_correction_used = false;
    attitude->snapshot.valid = true;
    return true;
}

bool Algorithm_Attitude_SetGyroBias(
    Algorithm_Attitude *attitude,
    const float bias_rad_per_second[3])
{
    unsigned int axis;

    if ((attitude == NULL) || !attitude->initialized ||
        !finite3(bias_rad_per_second)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        attitude->config.gyro_bias_rad_per_second[axis] =
            bias_rad_per_second[axis];
        attitude->snapshot.gyro_bias_rad_per_second[axis] =
            bias_rad_per_second[axis];
        attitude->integral_feedback_rad_per_second[axis] = 0.0F;
    }
    return true;
}

bool Algorithm_Attitude_GetSnapshot(
    const Algorithm_Attitude *attitude,
    Algorithm_Attitude_Snapshot *snapshot)
{
    if ((attitude == NULL) || !attitude->initialized ||
        (snapshot == NULL)) {
        return false;
    }
    *snapshot = attitude->snapshot;
    return true;
}
