#ifndef DEVICE_SENSOR_IMU_DEVICE_IMU_H
#define DEVICE_SENSOR_IMU_DEVICE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "algorithm_attitude.h"

typedef enum {
    DEVICE_IMU_SENSOR_AXIS_X = 0,
    DEVICE_IMU_SENSOR_AXIS_Y,
    DEVICE_IMU_SENSOR_AXIS_Z,
} Device_IMU_SensorAxis;

/* One entry describes one body output axis. source selects the sensor axis;
 * direction must be +1 or -1. All three source axes must form a permutation.
 * The complete 3x3 signed-permutation matrix must have determinant +1, so the
 * body frame remains right-handed; determinant -1 reflections make Init fail. */
typedef struct {
    Device_IMU_SensorAxis source;
    int8_t direction;
} Device_IMU_AxisMapping;

typedef enum {
    DEVICE_IMU_CALIBRATION_IDLE = 0,
    DEVICE_IMU_CALIBRATION_COLLECTING,
    DEVICE_IMU_CALIBRATION_COMPLETE,
} Device_IMU_CalibrationStatus;

typedef struct {
    Device_IMU_AxisMapping body_axis[3];
    float acceleration_mps2_per_sensor_unit;
    float angular_rate_dps_per_sensor_unit;
    float initial_gyro_bias_dps[3];
    bool initial_gyro_bias_valid;
    bool start_calibration_on_init;
    uint32_t offline_timeout_ms;
    uint32_t calibration_required_samples;
    float stationary_acceleration_tolerance_mps2;
    float stationary_gyro_maximum_dps;
    Algorithm_Attitude_Config attitude;
} Device_IMU_Config;

typedef struct {
    float acceleration_sensor_units[3];
    float angular_rate_sensor_units[3];
    uint32_t sampled_at_ms;
    float dt_seconds;
} Device_IMU_Sample;

typedef struct {
    float acceleration_mps2[3];
    float angular_rate_uncorrected_dps[3];
    float angular_rate_dps[3];
    float gyro_bias_dps[3];
    Algorithm_Attitude_Snapshot attitude;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    uint32_t sampled_at_ms;
    uint32_t sample_count;
    uint32_t calibration_samples_collected;
    uint32_t calibration_motion_rejections;
    Device_IMU_CalibrationStatus calibration_status;
    bool gyro_bias_valid;
    bool has_sample;
    bool online;
    bool feedback_valid;
} Device_IMU_Snapshot;

typedef struct {
    Device_IMU_Config config;
    Algorithm_Attitude attitude;
    Device_IMU_Snapshot snapshot;
    float calibration_sum_dps[3];
    bool initialized;
} Device_IMU;

Device_IMU_Config Device_IMU_DefaultConfig(void);
bool Device_IMU_Init(
    Device_IMU *device, const Device_IMU_Config *config);

/* The sample is copied and converted immediately. It may contain physical
 * units directly by keeping both configured scales at 1. Axis mapping then
 * produces the right-handed body frame consumed by Algorithm_Attitude.
 * After the first accepted sample, sampled_at_ms must be strictly newer under
 * uint32 serial arithmetic: (new - previous) must be 1..UINT32_MAX/2.
 * This accepts normal counter wrap. Delta 0, the ambiguous delta 2^31, and
 * values in the backward half-range are rejected without changing state. */
bool Device_IMU_PushSample(
    Device_IMU *device,
    const Device_IMU_Sample *sample);

/* Calibration collects consecutive samples that satisfy both stationary
 * thresholds. Motion resets the partial sum. Existing bias remains active
 * until a new complete mean is committed. */
bool Device_IMU_StartGyroBiasCalibration(Device_IMU *device);
bool Device_IMU_CancelGyroBiasCalibration(Device_IMU *device);
bool Device_IMU_SetGyroBiasDps(
    Device_IMU *device, const float bias_dps[3]);

/* Snapshot freshness uses unsigned elapsed time and supports uint32_t wrap.
 * feedback_valid requires a sample, an online timestamp, and valid attitude. */
bool Device_IMU_GetSnapshot(
    const Device_IMU *device,
    uint32_t now_ms,
    Device_IMU_Snapshot *snapshot);

#endif
