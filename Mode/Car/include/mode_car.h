#ifndef MODE_CAR_H
#define MODE_CAR_H

#include "algorithm_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t now_ms;
    float dt_seconds;
} Mode_Car_TimeInput;

typedef enum {
    MODE_CAR_COMPLETION_NONE = 0,
    MODE_CAR_COMPLETION_TARGET_REACHED,
    MODE_CAR_COMPLETION_STOP_STABLE,
    MODE_CAR_COMPLETION_TIMEOUT,
    MODE_CAR_COMPLETION_TERMINAL,
} Mode_Car_Completion;

/* These values are drive intentions, not permission to energize a motor.
 * APP remains responsible for the motor lock, limits, fault handling and the
 * final mapping to Device/BSP commands. */
typedef struct {
    float left_drive;
    float right_drive;
    bool motion_requested;
    Mode_Car_Completion completion;
} Mode_Car_Output;

typedef struct {
    bool initialized;
} Mode_Car_IdleContext;

bool Mode_Car_Idle_Init(Mode_Car_IdleContext *context);
bool Mode_Car_Idle_Update(
    const Mode_Car_IdleContext *context,
    const Mode_Car_TimeInput *input,
    Mode_Car_Output *output);

typedef struct {
    float kp;
    float ki;
    float kd;
    /* Legacy gains were tuned per control update. The reference period keeps
     * their nominal 5 ms response while Update still uses real elapsed time. */
    float gain_reference_seconds;
    /* Absolute limit of the I contribution, not the raw integral state. */
    float integral_output_limit;
    float maximum_correction;
    float correction_ratio;
} Mode_Car_StraightConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float base_drive;
    float target_heading_deg;
    float measured_heading_deg;
} Mode_Car_StraightInput;

typedef struct {
    Mode_Car_StraightConfig config;
    Algorithm_Control_PID heading_pid;
    bool initialized;
} Mode_Car_StraightContext;

Mode_Car_StraightConfig Mode_Car_Straight_DefaultConfig(void);
bool Mode_Car_Straight_Init(
    Mode_Car_StraightContext *context,
    const Mode_Car_StraightConfig *config);
bool Mode_Car_Straight_Update(
    Mode_Car_StraightContext *context,
    const Mode_Car_StraightInput *input,
    Mode_Car_Output *output);

typedef struct {
    float kp;
    float ki;
    float kd;
    float gain_reference_seconds;
    float integral_output_limit;
    float maximum_turn_drive;
    float minimum_turn_drive;
    float deceleration_zone_deg;
    float arrival_tolerance_deg;
} Mode_Car_TurningConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float target_heading_deg;
    float measured_heading_deg;
} Mode_Car_TurningInput;

typedef struct {
    Mode_Car_TurningConfig config;
    Algorithm_Control_PID heading_pid;
    Mode_Car_Completion completion;
    bool initialized;
} Mode_Car_TurningContext;

Mode_Car_TurningConfig Mode_Car_Turning_DefaultConfig(void);
bool Mode_Car_Turning_Init(
    Mode_Car_TurningContext *context,
    const Mode_Car_TurningConfig *config);
bool Mode_Car_Turning_Update(
    Mode_Car_TurningContext *context,
    const Mode_Car_TurningInput *input,
    Mode_Car_Output *output);

typedef struct {
    /* Absolute output limit in the caller's drive unit. */
    float maximum_drive;
} Mode_Car_SpinConfig;

typedef struct {
    Mode_Car_TimeInput time;
    /* Positive: left wheel reverse, right wheel forward. */
    float spin_drive;
} Mode_Car_SpinInput;

typedef struct {
    Mode_Car_SpinConfig config;
    bool initialized;
} Mode_Car_SpinContext;

Mode_Car_SpinConfig Mode_Car_Spin_DefaultConfig(void);
bool Mode_Car_Spin_Init(
    Mode_Car_SpinContext *context,
    const Mode_Car_SpinConfig *config);
bool Mode_Car_Spin_Update(
    const Mode_Car_SpinContext *context,
    const Mode_Car_SpinInput *input,
    Mode_Car_Output *output);

typedef struct {
    float kp;
    float ki;
    float integral_output_limit;
    float maximum_correction_drive;
    float feedforward_drive_per_dps;
    float maximum_drive;
} Mode_Car_SpinRateConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float target_yaw_rate_deg_per_second;
    float measured_yaw_rate_deg_per_second;
} Mode_Car_SpinRateInput;

typedef struct {
    Mode_Car_SpinRateConfig config;
    Algorithm_Control_PID rate_pid;
    bool initialized;
} Mode_Car_SpinRateContext;

Mode_Car_SpinRateConfig Mode_Car_SpinRate_DefaultConfig(void);
bool Mode_Car_SpinRate_Init(
    Mode_Car_SpinRateContext *context,
    const Mode_Car_SpinRateConfig *config);
void Mode_Car_SpinRate_Reset(Mode_Car_SpinRateContext *context);
bool Mode_Car_SpinRate_Update(
    Mode_Car_SpinRateContext *context,
    const Mode_Car_SpinRateInput *input,
    Mode_Car_Output *output);

typedef struct {
    float outer_kp;
    float outer_ki;
    float outer_integral_output_limit;
    float maximum_heading_rate_deg_per_second;
    float outer_period_seconds;
    float inner_kp;
    float inner_ki;
    float inner_integral_output_limit;
    float maximum_drive_difference;
    float drive_difference_slew_per_second;
} Mode_Car_CurveConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float base_drive;
    float target_heading_deg;
    float measured_heading_deg;
    /* Positive rate must mean increasing measured_heading_deg. */
    float measured_heading_rate_deg_per_second;
} Mode_Car_CurveInput;

typedef struct {
    Mode_Car_CurveConfig config;
    Algorithm_Control_PID heading_pid;
    Algorithm_Control_PID heading_rate_pid;
    float target_heading_rate_deg_per_second;
    float drive_difference;
    float outer_elapsed_seconds;
    bool outer_initialized;
    bool initialized;
} Mode_Car_CurveContext;

Mode_Car_CurveConfig Mode_Car_Curve_DefaultConfig(void);
bool Mode_Car_Curve_Init(
    Mode_Car_CurveContext *context,
    const Mode_Car_CurveConfig *config);
bool Mode_Car_Curve_Update(
    Mode_Car_CurveContext *context,
    const Mode_Car_CurveInput *input,
    Mode_Car_Output *output);

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_output_limit;
    float maximum_correction;
    float maximum_drive;
    float error_deadband;
    bool stop_on_intersection;
} Mode_Car_TrackingConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float base_drive;
    /* Positive means the line is to the right of the desired position. */
    float line_error;
    bool line_valid;
    bool line_lost;
    bool intersection;
} Mode_Car_TrackingInput;

typedef struct {
    Mode_Car_TrackingConfig config;
    Algorithm_Control_PID line_pid;
    bool initialized;
} Mode_Car_TrackingContext;

Mode_Car_TrackingConfig Mode_Car_Tracking_DefaultConfig(void);
bool Mode_Car_Tracking_Init(
    Mode_Car_TrackingContext *context,
    const Mode_Car_TrackingConfig *config);
bool Mode_Car_Tracking_Update(
    Mode_Car_TrackingContext *context,
    const Mode_Car_TrackingInput *input,
    Mode_Car_Output *output);

typedef struct {
    float stopped_speed_threshold;
    float stable_time_seconds;
    uint32_t timeout_ms;
} Mode_Car_BrakeConfig;

typedef struct {
    Mode_Car_TimeInput time;
    float measured_left_speed;
    float measured_right_speed;
} Mode_Car_BrakeInput;

typedef struct {
    Mode_Car_BrakeConfig config;
    uint32_t started_at_ms;
    float stable_elapsed_seconds;
    Mode_Car_Completion completion;
    bool timing_started;
    bool initialized;
} Mode_Car_BrakeContext;

Mode_Car_BrakeConfig Mode_Car_Brake_DefaultConfig(void);
bool Mode_Car_Brake_Init(
    Mode_Car_BrakeContext *context,
    const Mode_Car_BrakeConfig *config);
bool Mode_Car_Brake_Update(
    Mode_Car_BrakeContext *context,
    const Mode_Car_BrakeInput *input,
    Mode_Car_Output *output);

typedef struct {
    bool initialized;
} Mode_Car_DoneContext;

bool Mode_Car_Done_Init(Mode_Car_DoneContext *context);
bool Mode_Car_Done_Update(
    const Mode_Car_DoneContext *context,
    const Mode_Car_TimeInput *input,
    Mode_Car_Output *output);

#endif
