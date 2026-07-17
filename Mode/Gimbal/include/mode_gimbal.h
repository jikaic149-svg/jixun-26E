#ifndef MODE_GIMBAL_H
#define MODE_GIMBAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MODE_GIMBAL_VISION_PREDICTOR_ENABLED 0
#define MODE_GIMBAL_GRAVITY_POINTS_PER_PASS 9U
#define MODE_GIMBAL_GRAVITY_SAMPLE_CAPACITY \
    (2U * MODE_GIMBAL_GRAVITY_POINTS_PER_PASS)

typedef struct {
    uint32_t now_ms;
    float dt_seconds;
} Mode_Gimbal_TimeInput;

/* Values are current intentions in product-defined current units. They are
 * never permission to energize hardware. The product Mode owns the final
 * lock, feedback timeout, emergency stop, physical soft limits, and clamp.
 */
typedef struct {
    float pitch_current;
    float yaw_current;
    bool current_requested;
} Mode_Gimbal_CurrentIntent;

typedef struct {
    bool encoder_feedback_valid;
    bool imu_feedback_valid;
    float pitch_encoder_deg;
    float yaw_encoder_deg;
    float pitch_motor_speed_deg_per_second;
    float yaw_motor_speed_deg_per_second;
    float measured_pitch_current;
    float imu_pitch_deg;
    float imu_yaw_deg;
    float imu_pitch_rate_deg_per_second;
    float imu_yaw_rate_deg_per_second;
} Mode_Gimbal_FeedbackInput;

typedef struct {
    float kp;
    float ki;
    float kd_speed;
    float polarity;
    float deadband_deg;
    float maximum_error_deg;
    float integral_separation_deg;
    float integral_limit;
    float output_limit;
    float feedforward_threshold_deg;
    float positive_feedforward;
    float negative_feedforward;
    bool wrap_error;
    bool target_limit_enabled;
    float target_minimum_deg;
    float target_maximum_deg;
} Mode_Gimbal_AngleLoopConfig;

typedef struct {
    float dt_seconds;
    float target_angle_deg;
    float measured_angle_deg;
    float measured_speed_deg_per_second;
} Mode_Gimbal_AngleLoopInput;

typedef struct {
    float current_intent;
    float effective_target_deg;
    float error_deg;
} Mode_Gimbal_AngleLoopOutput;

typedef struct {
    Mode_Gimbal_AngleLoopConfig config;
    float integral_current;
    bool initialized;
} Mode_Gimbal_AngleLoopContext;

Mode_Gimbal_AngleLoopConfig
Mode_Gimbal_AngleLoop_DefaultPitchConfig(void);
Mode_Gimbal_AngleLoopConfig
Mode_Gimbal_AngleLoop_DefaultYawConfig(void);
bool Mode_Gimbal_AngleLoop_Init(
    Mode_Gimbal_AngleLoopContext *context,
    const Mode_Gimbal_AngleLoopConfig *config);
bool Mode_Gimbal_AngleLoop_Update(
    Mode_Gimbal_AngleLoopContext *context,
    const Mode_Gimbal_AngleLoopInput *input,
    Mode_Gimbal_AngleLoopOutput *output);

typedef struct {
    Mode_Gimbal_AngleLoopConfig pitch_loop;
    Mode_Gimbal_AngleLoopConfig yaw_loop;
    uint32_t feedback_timeout_ms;
} Mode_Gimbal_StabCalibConfig;

/* Deliberately encoder-only. Gyroscope bias sampling belongs to a separate
 * sensor calibration component and is not claimed by this mode. */
typedef struct {
    Mode_Gimbal_TimeInput time;
    bool encoder_feedback_valid;
    float pitch_encoder_deg;
    float yaw_encoder_deg;
    float pitch_motor_speed_deg_per_second;
    float yaw_motor_speed_deg_per_second;
} Mode_Gimbal_StabCalibInput;

typedef enum {
    MODE_GIMBAL_STAB_CALIB_WAITING_FEEDBACK = 0,
    MODE_GIMBAL_STAB_CALIB_ENCODERS_LOCKED,
    MODE_GIMBAL_STAB_CALIB_FEEDBACK_LOST,
    MODE_GIMBAL_STAB_CALIB_FEEDBACK_TIMEOUT
} Mode_Gimbal_StabCalibStatus;

typedef struct {
    Mode_Gimbal_CurrentIntent current;
    Mode_Gimbal_StabCalibStatus status;
    float locked_pitch_deg;
    float locked_yaw_deg;
} Mode_Gimbal_StabCalibOutput;

typedef struct {
    Mode_Gimbal_StabCalibConfig config;
    Mode_Gimbal_AngleLoopContext pitch_loop;
    Mode_Gimbal_AngleLoopContext yaw_loop;
    uint32_t waiting_started_at_ms;
    float locked_pitch_deg;
    float locked_yaw_deg;
    bool waiting_started;
    bool target_locked;
    bool timed_out;
    bool initialized;
} Mode_Gimbal_StabCalibContext;

Mode_Gimbal_StabCalibConfig Mode_Gimbal_StabCalib_DefaultConfig(void);
bool Mode_Gimbal_StabCalib_Init(
    Mode_Gimbal_StabCalibContext *context,
    const Mode_Gimbal_StabCalibConfig *config);
bool Mode_Gimbal_StabCalib_Update(
    Mode_Gimbal_StabCalibContext *context,
    const Mode_Gimbal_StabCalibInput *input,
    Mode_Gimbal_StabCalibOutput *output);

typedef struct {
    float pitch_kp;
    float pitch_ki;
    float pitch_kd_speed;
    float pitch_polarity;
    float pitch_deadband_deg;
    float pitch_maximum_error_deg;
    float pitch_integral_limit;
    float pitch_output_limit;
    float pitch_speed_filter_old_weight;
    float yaw_encoder_from_imu_polarity;
    Mode_Gimbal_AngleLoopConfig yaw_loop;
} Mode_Gimbal_GimbalStabConfig;

typedef struct {
    Mode_Gimbal_TimeInput time;
    Mode_Gimbal_FeedbackInput feedback;
    bool target_valid;
    float target_pitch_deg;
    float target_yaw_deg;
} Mode_Gimbal_GimbalStabInput;

typedef struct {
    Mode_Gimbal_CurrentIntent current;
    bool feedback_ready;
    float target_pitch_deg;
    float target_yaw_deg;
    float pitch_error_deg;
    float yaw_encoder_target_deg;
} Mode_Gimbal_GimbalStabOutput;

typedef struct {
    Mode_Gimbal_GimbalStabConfig config;
    Mode_Gimbal_AngleLoopContext yaw_loop;
    float target_pitch_deg;
    float target_yaw_deg;
    float pitch_integral_current;
    float filtered_pitch_rate_deg_per_second;
    bool target_initialized;
    bool initialized;
} Mode_Gimbal_GimbalStabContext;

Mode_Gimbal_GimbalStabConfig Mode_Gimbal_GimbalStab_DefaultConfig(void);
bool Mode_Gimbal_GimbalStab_Init(
    Mode_Gimbal_GimbalStabContext *context,
    const Mode_Gimbal_GimbalStabConfig *config);
bool Mode_Gimbal_GimbalStab_Update(
    Mode_Gimbal_GimbalStabContext *context,
    const Mode_Gimbal_GimbalStabInput *input,
    Mode_Gimbal_GimbalStabOutput *output);

typedef struct {
    float kp;
    /* Discrete gains are normalized to nominal_control_period_seconds. */
    float ki_per_nominal_step;
    float kd_per_nominal_step;
    float integral_limit;
    float maximum_step_deg;
} Mode_Gimbal_VisionAxisTuning;

typedef struct {
    bool available;
    bool valid;
    bool circle_target;
    uint32_t sequence;
    uint32_t received_at_ms;
    float dx_pixels;
    float dy_pixels;
} Mode_Gimbal_VisionSample;

/* dx/dy use a physical control convention: a positive error requests an
 * increase of the corresponding yaw/pitch target. Device/Task converts
 * camera coordinates, including image Y-down, before entering Mode. */

typedef struct {
    Mode_Gimbal_GimbalStabConfig stabilization;
    float nominal_control_period_seconds;
    uint32_t input_timeout_ms;
    float maximum_dx_pixels;
    float maximum_dy_pixels;
    float yaw_deadband_pixels;
    float pitch_deadband_pixels;
    float yaw_integral_deadband_pixels;
    float pitch_integral_deadband_pixels;
    float center_integral_decay;
    float reversal_integral_decay;
    float input_filter_alpha;
    float yaw_pixel_polarity;
    float pitch_pixel_polarity;
    Mode_Gimbal_VisionAxisTuning normal_yaw;
    Mode_Gimbal_VisionAxisTuning normal_pitch;
    Mode_Gimbal_VisionAxisTuning circle_yaw;
    Mode_Gimbal_VisionAxisTuning circle_pitch;
    uint32_t lost_scan_delay_ms;
    float lost_scan_half_span_deg;
    float lost_scan_rate_deg_per_second;
} Mode_Gimbal_VisionStabConfig;

typedef struct {
    Mode_Gimbal_TimeInput time;
    Mode_Gimbal_FeedbackInput feedback;
    Mode_Gimbal_VisionSample vision;
} Mode_Gimbal_VisionStabInput;

typedef struct {
    Mode_Gimbal_CurrentIntent current;
    bool feedback_ready;
    bool vision_active;
    bool scanning;
    float filtered_dx_pixels;
    float filtered_dy_pixels;
    float pitch_offset_deg;
    float yaw_offset_deg;
} Mode_Gimbal_VisionStabOutput;

typedef struct {
    Mode_Gimbal_VisionStabConfig config;
    Mode_Gimbal_GimbalStabContext stabilization;
    float entry_pitch_deg;
    float entry_yaw_deg;
    float target_pitch_deg;
    float target_yaw_deg;
    float filtered_dx_pixels;
    float filtered_dy_pixels;
    float yaw_integral;
    float pitch_integral;
    float previous_yaw_error;
    float previous_pitch_error;
    float control_elapsed_seconds;
    float scan_center_yaw_deg;
    int8_t scan_direction;
    uint32_t last_sequence;
    uint32_t loss_started_at_ms;
    bool target_initialized;
    bool filter_initialized;
    bool sequence_seen;
    bool previous_error_valid;
    bool loss_started;
    bool initialized;
} Mode_Gimbal_VisionStabContext;

Mode_Gimbal_VisionStabConfig Mode_Gimbal_VisionStab_DefaultConfig(void);
bool Mode_Gimbal_VisionStab_Init(
    Mode_Gimbal_VisionStabContext *context,
    const Mode_Gimbal_VisionStabConfig *config);
bool Mode_Gimbal_VisionStab_Update(
    Mode_Gimbal_VisionStabContext *context,
    const Mode_Gimbal_VisionStabInput *input,
    Mode_Gimbal_VisionStabOutput *output);

typedef struct {
    float sin_coefficient;
    float cos_coefficient;
    float offset;
    float maximum_current;
} Mode_Gimbal_GravityFeedforwardConfig;

typedef struct {
    Mode_Gimbal_AngleLoopConfig pitch_loop;
    Mode_Gimbal_AngleLoopConfig yaw_loop;
    float pitch_minimum_deg;
    float pitch_maximum_deg;
    float endpoint_margin_deg;
    uint32_t settle_time_ms;
    uint32_t sample_time_ms;
    Mode_Gimbal_GravityFeedforwardConfig feedforward;
} Mode_Gimbal_GravityCalibConfig;

typedef struct {
    float encoder_angle_deg;
    float measured_current;
} Mode_Gimbal_GravitySample;

typedef enum {
    MODE_GIMBAL_GRAVITY_WAITING_FEEDBACK = 0,
    MODE_GIMBAL_GRAVITY_SETTLING,
    MODE_GIMBAL_GRAVITY_SAMPLING,
    MODE_GIMBAL_GRAVITY_SAMPLE_READY,
    MODE_GIMBAL_GRAVITY_COMPLETE
} Mode_Gimbal_GravityCalibStatus;

typedef struct {
    Mode_Gimbal_TimeInput time;
    bool encoder_feedback_valid;
    float pitch_encoder_deg;
    float yaw_encoder_deg;
    float pitch_motor_speed_deg_per_second;
    float yaw_motor_speed_deg_per_second;
    float measured_pitch_current;
} Mode_Gimbal_GravityCalibInput;

typedef struct {
    Mode_Gimbal_CurrentIntent current;
    Mode_Gimbal_GravityCalibStatus status;
    size_t sample_count;
    size_t active_sample_index;
    float target_pitch_deg;
    bool sample_ready;
    Mode_Gimbal_GravitySample sample;
} Mode_Gimbal_GravityCalibOutput;

typedef struct {
    Mode_Gimbal_GravityCalibConfig config;
    Mode_Gimbal_AngleLoopContext pitch_loop;
    Mode_Gimbal_AngleLoopContext yaw_loop;
    Mode_Gimbal_GravitySample samples[
        MODE_GIMBAL_GRAVITY_SAMPLE_CAPACITY];
    size_t sample_count;
    uint32_t phase_started_at_ms;
    float yaw_target_deg;
    float accumulated_angle_seconds;
    float accumulated_current_seconds;
    float accumulated_seconds;
    Mode_Gimbal_GravityCalibStatus phase;
    bool target_initialized;
    bool phase_timing_started;
    bool initialized;
} Mode_Gimbal_GravityCalibContext;

Mode_Gimbal_GravityFeedforwardConfig
Mode_Gimbal_GravityFeedforward_DefaultConfig(void);
bool Mode_Gimbal_GravityFeedforward_Calculate(
    const Mode_Gimbal_GravityFeedforwardConfig *config,
    float pitch_encoder_deg,
    float *current_intent);
Mode_Gimbal_GravityCalibConfig Mode_Gimbal_GravityCalib_DefaultConfig(void);
bool Mode_Gimbal_GravityCalib_Init(
    Mode_Gimbal_GravityCalibContext *context,
    const Mode_Gimbal_GravityCalibConfig *config);
bool Mode_Gimbal_GravityCalib_Update(
    Mode_Gimbal_GravityCalibContext *context,
    const Mode_Gimbal_GravityCalibInput *input,
    Mode_Gimbal_GravityCalibOutput *output);
size_t Mode_Gimbal_GravityCalib_SampleCount(
    const Mode_Gimbal_GravityCalibContext *context);
bool Mode_Gimbal_GravityCalib_GetSample(
    const Mode_Gimbal_GravityCalibContext *context,
    size_t index,
    Mode_Gimbal_GravitySample *sample);

typedef enum {
    MODE_GIMBAL_SCHEDULER_IDLE = 0,
    MODE_GIMBAL_SCHEDULER_RELAX,
    MODE_GIMBAL_SCHEDULER_ENCODER_LOCK,
    MODE_GIMBAL_SCHEDULER_GIMBAL_STAB,
    MODE_GIMBAL_SCHEDULER_VISION_STAB,
    MODE_GIMBAL_SCHEDULER_GRAVITY_CALIB
} Mode_Gimbal_SchedulerMode;

typedef struct {
    Mode_Gimbal_StabCalibConfig encoder_lock;
    Mode_Gimbal_GimbalStabConfig gimbal_stab;
    Mode_Gimbal_VisionStabConfig vision_stab;
    Mode_Gimbal_GravityCalibConfig gravity_calib;
} Mode_Gimbal_SchedulerConfig;

typedef struct {
    Mode_Gimbal_SchedulerMode requested_mode;
    Mode_Gimbal_TimeInput time;
    Mode_Gimbal_FeedbackInput feedback;
    bool gimbal_target_valid;
    float gimbal_target_pitch_deg;
    float gimbal_target_yaw_deg;
    Mode_Gimbal_VisionSample vision;
} Mode_Gimbal_SchedulerInput;

typedef struct {
    Mode_Gimbal_CurrentIntent current;
    Mode_Gimbal_SchedulerMode active_mode;
    bool mode_entered;
    Mode_Gimbal_StabCalibStatus encoder_lock_status;
    bool vision_active;
    bool vision_scanning;
    Mode_Gimbal_GravityCalibStatus gravity_status;
    bool gravity_sample_ready;
    Mode_Gimbal_GravitySample gravity_sample;
} Mode_Gimbal_SchedulerOutput;

typedef struct {
    Mode_Gimbal_SchedulerConfig config;
    Mode_Gimbal_StabCalibContext encoder_lock;
    Mode_Gimbal_GimbalStabContext gimbal_stab;
    Mode_Gimbal_VisionStabContext vision_stab;
    Mode_Gimbal_GravityCalibContext gravity_calib;
    Mode_Gimbal_SchedulerMode active_mode;
    bool active_mode_valid;
    bool initialized;
} Mode_Gimbal_SchedulerContext;

Mode_Gimbal_SchedulerConfig Mode_Gimbal_Scheduler_DefaultConfig(void);
bool Mode_Gimbal_Scheduler_Init(
    Mode_Gimbal_SchedulerContext *context,
    const Mode_Gimbal_SchedulerConfig *config);
bool Mode_Gimbal_Scheduler_Update(
    Mode_Gimbal_SchedulerContext *context,
    const Mode_Gimbal_SchedulerInput *input,
    Mode_Gimbal_SchedulerOutput *output);

#endif
