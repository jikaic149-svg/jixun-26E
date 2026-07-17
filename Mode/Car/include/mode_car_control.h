#ifndef MODE_CAR_CONTROL_H
#define MODE_CAR_CONTROL_H

#include "algorithm_control.h"

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} Mode_Car_SpeedBandConfig;

typedef struct {
    Mode_Car_SpeedBandConfig low_speed;
    Mode_Car_SpeedBandConfig high_speed;
    float high_speed_threshold;
    float error_deadband;
    float feedforward_percent_per_speed;
    float target_smoothing_time_constant_seconds;
    float maximum_target_speed;
    float maximum_effort_percent;
    float stopped_speed_threshold;
} Mode_Car_SpeedControllerConfig;

typedef enum {
    MODE_CAR_SPEED_RANGE_LOW = 0,
    MODE_CAR_SPEED_RANGE_HIGH,
} Mode_Car_SpeedRange;

typedef struct {
    float smoothed_target;
    /* Integral is stored directly as motor-effort percent.  This lets the
     * speed controller apply anti-windup against the final motor command,
     * including feedforward, instead of clamping an isolated PID term. */
    float integral_effort_percent;
    float previous_measurement;
    Mode_Car_SpeedRange range;
    bool measurement_initialized;
} Mode_Car_WheelController;

typedef struct {
    Mode_Car_SpeedControllerConfig config;
    Mode_Car_WheelController left;
    Mode_Car_WheelController right;
    bool initialized;
} Mode_Car_SpeedController;

typedef struct {
    float dt_seconds;
    float target_left_speed;
    float target_right_speed;
    float measured_left_speed;
    float measured_right_speed;
} Mode_Car_SpeedControllerInput;

typedef struct {
    float smoothed_left_target;
    float smoothed_right_target;
    float left_effort_percent;
    float right_effort_percent;
    Mode_Car_SpeedRange left_range;
    Mode_Car_SpeedRange right_range;
} Mode_Car_SpeedControllerOutput;

Mode_Car_SpeedControllerConfig
Mode_Car_SpeedController_DefaultConfig(void);
bool Mode_Car_SpeedController_Init(
    Mode_Car_SpeedController *controller,
    const Mode_Car_SpeedControllerConfig *config);
void Mode_Car_SpeedController_Reset(
    Mode_Car_SpeedController *controller);
bool Mode_Car_SpeedController_Update(
    Mode_Car_SpeedController *controller,
    const Mode_Car_SpeedControllerInput *input,
    Mode_Car_SpeedControllerOutput *output);

typedef struct {
    /* Outer-loop output is a differential wheel-speed target in mm/s. */
    float kp;
    float ki;
    /* D term uses measured yaw rate directly to avoid wrap differentiation. */
    float kd;
    float integral_speed_limit;
    float maximum_wheel_speed;
    float angle_tolerance_deg;
    float rate_tolerance_deg_per_second;
} Mode_Car_AngleControllerConfig;

typedef struct {
    Mode_Car_AngleControllerConfig config;
    float integral_wheel_speed;
    bool initialized;
} Mode_Car_AngleController;

typedef struct {
    float dt_seconds;
    float target_yaw_deg;
    float measured_yaw_deg;
    float measured_yaw_rate_deg_per_second;
} Mode_Car_AngleControllerInput;

typedef struct {
    float angle_error_deg;
    float turn_wheel_speed;
    float target_left_speed;
    float target_right_speed;
    bool within_tolerance;
} Mode_Car_AngleControllerOutput;

Mode_Car_AngleControllerConfig
Mode_Car_AngleController_DefaultConfig(void);
bool Mode_Car_AngleController_Init(
    Mode_Car_AngleController *controller,
    const Mode_Car_AngleControllerConfig *config);
void Mode_Car_AngleController_Reset(
    Mode_Car_AngleController *controller);
bool Mode_Car_AngleController_Update(
    Mode_Car_AngleController *controller,
    const Mode_Car_AngleControllerInput *input,
    Mode_Car_AngleControllerOutput *output);

#endif
