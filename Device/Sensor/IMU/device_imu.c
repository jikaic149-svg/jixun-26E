#include "device_imu.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define DEVICE_IMU_PI 3.14159265358979323846F
#define DEVICE_IMU_DEG_TO_RAD (DEVICE_IMU_PI / 180.0F)

static bool Device_IMU_Finite3(const float values[3])
{
    return (values != NULL) && isfinite(values[0]) &&
        isfinite(values[1]) && isfinite(values[2]);
}

static uint32_t Device_IMU_Increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : value + 1U;
}

static bool Device_IMU_ValidMapping(
    const Device_IMU_AxisMapping mapping[3])
{
    bool used[3] = {false, false, false};
    int determinant;
    unsigned int body_axis;
    unsigned int other_axis;

    if (mapping == NULL) {
        return false;
    }
    for (body_axis = 0U; body_axis < 3U; ++body_axis) {
        const unsigned int source = (unsigned int)mapping[body_axis].source;

        if ((source >= 3U) || used[source] ||
            ((mapping[body_axis].direction != 1) &&
             (mapping[body_axis].direction != -1))) {
            return false;
        }
        used[source] = true;
    }

    determinant = (int)mapping[0].direction *
        (int)mapping[1].direction * (int)mapping[2].direction;
    for (body_axis = 0U; body_axis < 3U; ++body_axis) {
        for (other_axis = body_axis + 1U;
             other_axis < 3U; ++other_axis) {
            if ((unsigned int)mapping[body_axis].source >
                (unsigned int)mapping[other_axis].source) {
                determinant = -determinant;
            }
        }
    }
    return determinant == 1;
}

static bool Device_IMU_MapVector(
    const Device_IMU_AxisMapping mapping[3],
    const float input[3],
    float scale,
    float output[3])
{
    unsigned int body_axis;

    for (body_axis = 0U; body_axis < 3U; ++body_axis) {
        const unsigned int source = (unsigned int)mapping[body_axis].source;

        output[body_axis] = input[source] * scale *
            (float)mapping[body_axis].direction;
        if (!isfinite(output[body_axis])) {
            return false;
        }
    }
    return true;
}

static bool Device_IMU_SetAttitudeBias(
    Algorithm_Attitude *attitude, const float bias_dps[3])
{
    float bias_rad_per_second[3];
    unsigned int axis;

    for (axis = 0U; axis < 3U; ++axis) {
        bias_rad_per_second[axis] =
            bias_dps[axis] * DEVICE_IMU_DEG_TO_RAD;
        if (!isfinite(bias_rad_per_second[axis])) {
            return false;
        }
    }
    return Algorithm_Attitude_SetGyroBias(
        attitude, bias_rad_per_second);
}

static bool Device_IMU_Stationary(
    const Device_IMU *device,
    const float acceleration_mps2[3],
    const float angular_rate_dps[3])
{
    float acceleration_norm_squared;
    float gyro_norm_squared;
    float acceleration_norm;
    float gyro_norm;

    acceleration_norm_squared =
        (acceleration_mps2[0] * acceleration_mps2[0]) +
        (acceleration_mps2[1] * acceleration_mps2[1]) +
        (acceleration_mps2[2] * acceleration_mps2[2]);
    gyro_norm_squared =
        (angular_rate_dps[0] * angular_rate_dps[0]) +
        (angular_rate_dps[1] * angular_rate_dps[1]) +
        (angular_rate_dps[2] * angular_rate_dps[2]);
    if (!isfinite(acceleration_norm_squared) ||
        !isfinite(gyro_norm_squared)) {
        return false;
    }
    acceleration_norm = sqrtf(acceleration_norm_squared);
    gyro_norm = sqrtf(gyro_norm_squared);
    return isfinite(acceleration_norm) && isfinite(gyro_norm) &&
        (fabsf(acceleration_norm -
            device->config.attitude.acceleration_reference_mps2) <=
         device->config.stationary_acceleration_tolerance_mps2) &&
        (gyro_norm <= device->config.stationary_gyro_maximum_dps);
}

static void Device_IMU_ResetCalibrationAccumulator(Device_IMU *device)
{
    device->calibration_sum_dps[0] = 0.0F;
    device->calibration_sum_dps[1] = 0.0F;
    device->calibration_sum_dps[2] = 0.0F;
    device->snapshot.calibration_samples_collected = 0U;
}

Device_IMU_Config Device_IMU_DefaultConfig(void)
{
    Device_IMU_Config config;

    config.body_axis[0] = (Device_IMU_AxisMapping){
        DEVICE_IMU_SENSOR_AXIS_X, 1};
    config.body_axis[1] = (Device_IMU_AxisMapping){
        DEVICE_IMU_SENSOR_AXIS_Y, 1};
    config.body_axis[2] = (Device_IMU_AxisMapping){
        DEVICE_IMU_SENSOR_AXIS_Z, 1};
    config.acceleration_mps2_per_sensor_unit = 1.0F;
    config.angular_rate_dps_per_sensor_unit = 1.0F;
    config.initial_gyro_bias_dps[0] = 0.0F;
    config.initial_gyro_bias_dps[1] = 0.0F;
    config.initial_gyro_bias_dps[2] = 0.0F;
    config.initial_gyro_bias_valid = false;
    config.start_calibration_on_init = false;
    config.offline_timeout_ms = 100U;
    config.calibration_required_samples = 500U;
    config.stationary_acceleration_tolerance_mps2 = 0.75F;
    config.stationary_gyro_maximum_dps = 5.0F;
    config.attitude = Algorithm_Attitude_DefaultConfig();
    return config;
}

bool Device_IMU_Init(
    Device_IMU *device, const Device_IMU_Config *config)
{
    Device_IMU_Config selected = (config == NULL)
        ? Device_IMU_DefaultConfig()
        : *config;
    Algorithm_Attitude attitude;
    unsigned int axis;

    if (device != NULL) {
        *device = (Device_IMU){0};
    }
    if ((device == NULL) || !Device_IMU_ValidMapping(selected.body_axis) ||
        !isfinite(selected.acceleration_mps2_per_sensor_unit) ||
        !isfinite(selected.angular_rate_dps_per_sensor_unit) ||
        !Device_IMU_Finite3(selected.initial_gyro_bias_dps) ||
        !isfinite(selected.stationary_acceleration_tolerance_mps2) ||
        !isfinite(selected.stationary_gyro_maximum_dps) ||
        (selected.acceleration_mps2_per_sensor_unit <= 0.0F) ||
        (selected.angular_rate_dps_per_sensor_unit <= 0.0F) ||
        (selected.offline_timeout_ms == 0U) ||
        (selected.offline_timeout_ms > (UINT32_MAX / 2U)) ||
        (selected.calibration_required_samples == 0U) ||
        (selected.stationary_acceleration_tolerance_mps2 < 0.0F) ||
        (selected.stationary_gyro_maximum_dps < 0.0F)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        selected.attitude.gyro_bias_rad_per_second[axis] =
            selected.initial_gyro_bias_dps[axis] * DEVICE_IMU_DEG_TO_RAD;
        if (!isfinite(
                selected.attitude.gyro_bias_rad_per_second[axis])) {
            return false;
        }
    }
    if (!Algorithm_Attitude_Init(&attitude, &selected.attitude)) {
        return false;
    }

    device->config = selected;
    device->attitude = attitude;
    for (axis = 0U; axis < 3U; ++axis) {
        device->snapshot.gyro_bias_dps[axis] =
            selected.initial_gyro_bias_dps[axis];
    }
    device->snapshot.gyro_bias_valid = selected.initial_gyro_bias_valid;
    device->snapshot.calibration_status =
        selected.start_calibration_on_init
            ? DEVICE_IMU_CALIBRATION_COLLECTING
            : DEVICE_IMU_CALIBRATION_IDLE;
    device->snapshot.attitude = attitude.snapshot;
    device->initialized = true;
    return true;
}

bool Device_IMU_PushSample(
    Device_IMU *device,
    const Device_IMU_Sample *sample)
{
    Device_IMU next;
    Algorithm_Attitude_Input attitude_input;
    Algorithm_Attitude_Snapshot attitude_snapshot;
    float acceleration_mps2[3];
    float angular_rate_dps[3];
    unsigned int axis;

    if ((device == NULL) || !device->initialized || (sample == NULL) ||
        !Device_IMU_Finite3(sample->acceleration_sensor_units) ||
        !Device_IMU_Finite3(sample->angular_rate_sensor_units) ||
        !isfinite(sample->dt_seconds) || (sample->dt_seconds <= 0.0F)) {
        return false;
    }
    if (device->snapshot.has_sample) {
        const uint32_t timestamp_advance =
            sample->sampled_at_ms - device->snapshot.sampled_at_ms;

        /* RFC-1982-style serial ordering: only advances in the forward
         * half-range are newer. Zero and the ambiguous opposite half-range
         * are rejected along with older samples. */
        if ((timestamp_advance == 0U) ||
            (timestamp_advance > (UINT32_MAX / 2U))) {
            return false;
        }
    }
    if (!Device_IMU_MapVector(device->config.body_axis,
            sample->acceleration_sensor_units,
            device->config.acceleration_mps2_per_sensor_unit,
            acceleration_mps2) ||
        !Device_IMU_MapVector(device->config.body_axis,
            sample->angular_rate_sensor_units,
            device->config.angular_rate_dps_per_sensor_unit,
            angular_rate_dps)) {
        return false;
    }

    next = *device;
    if (next.snapshot.calibration_status ==
        DEVICE_IMU_CALIBRATION_COLLECTING) {
        if (!Device_IMU_Stationary(
                &next, acceleration_mps2, angular_rate_dps)) {
            Device_IMU_ResetCalibrationAccumulator(&next);
            next.snapshot.calibration_motion_rejections =
                Device_IMU_Increment(
                    next.snapshot.calibration_motion_rejections);
        }
        else {
            for (axis = 0U; axis < 3U; ++axis) {
                next.calibration_sum_dps[axis] += angular_rate_dps[axis];
                if (!isfinite(next.calibration_sum_dps[axis])) {
                    return false;
                }
            }
            next.snapshot.calibration_samples_collected =
                Device_IMU_Increment(
                    next.snapshot.calibration_samples_collected);
            if (next.snapshot.calibration_samples_collected >=
                next.config.calibration_required_samples) {
                const float divisor = (float)
                    next.snapshot.calibration_samples_collected;

                for (axis = 0U; axis < 3U; ++axis) {
                    next.snapshot.gyro_bias_dps[axis] =
                        next.calibration_sum_dps[axis] / divisor;
                    next.config.initial_gyro_bias_dps[axis] =
                        next.snapshot.gyro_bias_dps[axis];
                }
                if (!Device_IMU_SetAttitudeBias(
                        &next.attitude, next.snapshot.gyro_bias_dps)) {
                    return false;
                }
                next.snapshot.gyro_bias_valid = true;
                next.snapshot.calibration_status =
                    DEVICE_IMU_CALIBRATION_COMPLETE;
            }
        }
    }

    for (axis = 0U; axis < 3U; ++axis) {
        attitude_input.acceleration_mps2[axis] = acceleration_mps2[axis];
        attitude_input.angular_rate_rad_per_second[axis] =
            angular_rate_dps[axis] * DEVICE_IMU_DEG_TO_RAD;
        if (!isfinite(attitude_input.angular_rate_rad_per_second[axis])) {
            return false;
        }
    }
    attitude_input.dt_seconds = sample->dt_seconds;
    if (!Algorithm_Attitude_Update(
            &next.attitude, &attitude_input, &attitude_snapshot)) {
        return false;
    }

    for (axis = 0U; axis < 3U; ++axis) {
        next.snapshot.acceleration_mps2[axis] = acceleration_mps2[axis];
        next.snapshot.angular_rate_uncorrected_dps[axis] =
            angular_rate_dps[axis];
        next.snapshot.angular_rate_dps[axis] =
            angular_rate_dps[axis] - next.snapshot.gyro_bias_dps[axis];
        if (!isfinite(next.snapshot.angular_rate_dps[axis])) {
            return false;
        }
    }
    next.snapshot.attitude = attitude_snapshot;
    next.snapshot.roll_deg = attitude_snapshot.euler.roll_deg;
    next.snapshot.pitch_deg = attitude_snapshot.euler.pitch_deg;
    next.snapshot.yaw_deg = attitude_snapshot.euler.yaw_deg;
    next.snapshot.roll_rate_dps = next.snapshot.angular_rate_dps[0];
    next.snapshot.pitch_rate_dps = next.snapshot.angular_rate_dps[1];
    next.snapshot.yaw_rate_dps = next.snapshot.angular_rate_dps[2];
    next.snapshot.sampled_at_ms = sample->sampled_at_ms;
    next.snapshot.sample_count =
        Device_IMU_Increment(next.snapshot.sample_count);
    next.snapshot.has_sample = true;
    next.snapshot.online = true;
    next.snapshot.feedback_valid = attitude_snapshot.valid;
    *device = next;
    return true;
}

bool Device_IMU_StartGyroBiasCalibration(Device_IMU *device)
{
    if ((device == NULL) || !device->initialized) {
        return false;
    }
    Device_IMU_ResetCalibrationAccumulator(device);
    device->snapshot.calibration_motion_rejections = 0U;
    device->snapshot.calibration_status =
        DEVICE_IMU_CALIBRATION_COLLECTING;
    return true;
}

bool Device_IMU_CancelGyroBiasCalibration(Device_IMU *device)
{
    if ((device == NULL) || !device->initialized ||
        (device->snapshot.calibration_status !=
            DEVICE_IMU_CALIBRATION_COLLECTING)) {
        return false;
    }
    Device_IMU_ResetCalibrationAccumulator(device);
    device->snapshot.calibration_status = DEVICE_IMU_CALIBRATION_IDLE;
    return true;
}

bool Device_IMU_SetGyroBiasDps(
    Device_IMU *device, const float bias_dps[3])
{
    Algorithm_Attitude next_attitude;
    unsigned int axis;

    if ((device == NULL) || !device->initialized ||
        !Device_IMU_Finite3(bias_dps)) {
        return false;
    }
    next_attitude = device->attitude;
    if (!Device_IMU_SetAttitudeBias(&next_attitude, bias_dps)) {
        return false;
    }
    device->attitude = next_attitude;
    for (axis = 0U; axis < 3U; ++axis) {
        device->config.initial_gyro_bias_dps[axis] = bias_dps[axis];
        device->snapshot.gyro_bias_dps[axis] = bias_dps[axis];
    }
    device->config.initial_gyro_bias_valid = true;
    device->snapshot.gyro_bias_valid = true;
    Device_IMU_ResetCalibrationAccumulator(device);
    device->snapshot.calibration_status = DEVICE_IMU_CALIBRATION_COMPLETE;
    return true;
}

bool Device_IMU_GetSnapshot(
    const Device_IMU *device,
    uint32_t now_ms,
    Device_IMU_Snapshot *snapshot)
{
    if ((device == NULL) || !device->initialized || (snapshot == NULL)) {
        return false;
    }
    *snapshot = device->snapshot;
    snapshot->online = snapshot->has_sample &&
        ((uint32_t)(now_ms - snapshot->sampled_at_ms) <=
         device->config.offline_timeout_ms);
    snapshot->feedback_valid = snapshot->online &&
        snapshot->attitude.valid;
    return true;
}
