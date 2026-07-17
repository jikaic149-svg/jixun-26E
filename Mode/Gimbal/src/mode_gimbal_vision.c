#include "mode_gimbal_internal.h"

#include <math.h>
#include <stddef.h>

static bool valid_axis_tuning(const Mode_Gimbal_VisionAxisTuning *tuning)
{
    return (tuning != NULL) &&
        Mode_Gimbal_Internal_IsFinite(tuning->kp) &&
        Mode_Gimbal_Internal_IsFinite(tuning->ki_per_nominal_step) &&
        Mode_Gimbal_Internal_IsFinite(tuning->kd_per_nominal_step) &&
        Mode_Gimbal_Internal_IsFinite(tuning->integral_limit) &&
        Mode_Gimbal_Internal_IsFinite(tuning->maximum_step_deg) &&
        (tuning->integral_limit >= 0.0F) &&
        (tuning->maximum_step_deg > 0.0F);
}

static bool valid_vision_config(const Mode_Gimbal_VisionStabConfig *config)
{
    return (config != NULL) &&
        Mode_Gimbal_Internal_ValidGimbalConfig(&config->stabilization) &&
        Mode_Gimbal_Internal_IsFinite(
            config->nominal_control_period_seconds) &&
        (config->nominal_control_period_seconds > 0.0F) &&
        (config->input_timeout_ms > 0U) &&
        Mode_Gimbal_Internal_IsFinite(config->maximum_dx_pixels) &&
        Mode_Gimbal_Internal_IsFinite(config->maximum_dy_pixels) &&
        Mode_Gimbal_Internal_IsFinite(config->yaw_deadband_pixels) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_deadband_pixels) &&
        Mode_Gimbal_Internal_IsFinite(
            config->yaw_integral_deadband_pixels) &&
        Mode_Gimbal_Internal_IsFinite(
            config->pitch_integral_deadband_pixels) &&
        Mode_Gimbal_Internal_IsFinite(config->center_integral_decay) &&
        Mode_Gimbal_Internal_IsFinite(config->reversal_integral_decay) &&
        Mode_Gimbal_Internal_IsFinite(config->input_filter_alpha) &&
        Mode_Gimbal_Internal_IsFinite(config->yaw_pixel_polarity) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_pixel_polarity) &&
        Mode_Gimbal_Internal_IsFinite(config->lost_scan_half_span_deg) &&
        Mode_Gimbal_Internal_IsFinite(
            config->lost_scan_rate_deg_per_second) &&
        (config->maximum_dx_pixels > 0.0F) &&
        (config->maximum_dy_pixels > 0.0F) &&
        (config->yaw_deadband_pixels >= 0.0F) &&
        (config->pitch_deadband_pixels >= 0.0F) &&
        (config->yaw_integral_deadband_pixels >= 0.0F) &&
        (config->pitch_integral_deadband_pixels >= 0.0F) &&
        (config->center_integral_decay >= 0.0F) &&
        (config->center_integral_decay <= 1.0F) &&
        (config->reversal_integral_decay >= 0.0F) &&
        (config->reversal_integral_decay <= 1.0F) &&
        (config->input_filter_alpha > 0.0F) &&
        (config->input_filter_alpha <= 1.0F) &&
        (config->yaw_pixel_polarity != 0.0F) &&
        (config->pitch_pixel_polarity != 0.0F) &&
        (config->lost_scan_half_span_deg > 0.0F) &&
        (config->lost_scan_rate_deg_per_second > 0.0F) &&
        valid_axis_tuning(&config->normal_yaw) &&
        valid_axis_tuning(&config->normal_pitch) &&
        valid_axis_tuning(&config->circle_yaw) &&
        valid_axis_tuning(&config->circle_pitch);
}

static void vision_stop_output(Mode_Gimbal_VisionStabOutput *output)
{
    if (output == NULL) {
        return;
    }
    Mode_Gimbal_Internal_StopIntent(&output->current);
    output->feedback_ready = false;
    output->vision_active = false;
    output->scanning = false;
    output->filtered_dx_pixels = 0.0F;
    output->filtered_dy_pixels = 0.0F;
    output->pitch_offset_deg = 0.0F;
    output->yaw_offset_deg = 0.0F;
}

static float pixel_deadband(float value, float half_width)
{
    return (fabsf(value) <= half_width) ? 0.0F : value;
}

static bool update_vision_axis(
    const Mode_Gimbal_VisionAxisTuning *tuning,
    float error,
    float integral_deadband,
    float center_decay,
    float reversal_decay,
    float step_ratio,
    bool previous_valid,
    float previous_error,
    float *integral,
    float *step)
{
    float next_integral = *integral;
    float derivative = 0.0F;
    float next_step;

    if (fabsf(error) <= integral_deadband) {
        next_integral *= powf(center_decay, step_ratio);
    }
    else {
        if (previous_valid && ((error * previous_error) < 0.0F)) {
            next_integral *= reversal_decay;
        }
        next_integral += tuning->ki_per_nominal_step * error * step_ratio;
        next_integral = Mode_Gimbal_Internal_Clamp(next_integral,
            -tuning->integral_limit, tuning->integral_limit);
    }
    if (previous_valid && (step_ratio > 0.0F)) {
        derivative = (error - previous_error) / step_ratio;
    }
    next_step = (tuning->kp * error) + next_integral +
        (tuning->kd_per_nominal_step * derivative);
    if (!Mode_Gimbal_Internal_IsFinite(next_step) ||
        !Mode_Gimbal_Internal_IsFinite(next_integral)) {
        return false;
    }
    next_step = Mode_Gimbal_Internal_Clamp(next_step,
        -(tuning->maximum_step_deg * step_ratio),
        tuning->maximum_step_deg * step_ratio);
    *integral = next_integral;
    *step = next_step;
    return true;
}

static void update_scan_target(
    Mode_Gimbal_VisionStabContext *context, float dt_seconds)
{
    float relative = Mode_Gimbal_Internal_AngleError(
        context->target_yaw_deg, context->scan_center_yaw_deg);

    relative += (float) context->scan_direction *
        context->config.lost_scan_rate_deg_per_second * dt_seconds;
    if (relative >= context->config.lost_scan_half_span_deg) {
        relative = context->config.lost_scan_half_span_deg;
        context->scan_direction = -1;
    }
    else if (relative <= -context->config.lost_scan_half_span_deg) {
        relative = -context->config.lost_scan_half_span_deg;
        context->scan_direction = 1;
    }
    context->target_yaw_deg = Mode_Gimbal_Internal_NormalizeAngle(
        context->scan_center_yaw_deg + relative);
}

Mode_Gimbal_VisionStabConfig Mode_Gimbal_VisionStab_DefaultConfig(void)
{
    Mode_Gimbal_VisionStabConfig config;

    config.stabilization = Mode_Gimbal_GimbalStab_DefaultConfig();
    config.nominal_control_period_seconds = 0.010F;
    config.input_timeout_ms = 250U;
    config.maximum_dx_pixels = 180.0F;
    config.maximum_dy_pixels = 140.0F;
    config.yaw_deadband_pixels = 2.0F;
    config.pitch_deadband_pixels = 2.0F;
    config.yaw_integral_deadband_pixels = 1.0F;
    config.pitch_integral_deadband_pixels = 1.0F;
    config.center_integral_decay = 0.90F;
    config.reversal_integral_decay = 0.50F;
    config.input_filter_alpha = 0.72F;
    config.yaw_pixel_polarity = 1.0F;
    config.pitch_pixel_polarity = 1.0F;

    config.normal_yaw.kp = 0.00018F;
    config.normal_yaw.ki_per_nominal_step = 0.000002F;
    config.normal_yaw.kd_per_nominal_step = 0.0F;
    config.normal_yaw.integral_limit = 0.004F;
    config.normal_yaw.maximum_step_deg = 0.010F;
    config.normal_pitch.kp = 0.00018F;
    config.normal_pitch.ki_per_nominal_step = 0.000003F;
    config.normal_pitch.kd_per_nominal_step = 0.0F;
    config.normal_pitch.integral_limit = 0.006F;
    config.normal_pitch.maximum_step_deg = 0.012F;

    config.circle_yaw.kp = 0.00024F;
    config.circle_yaw.ki_per_nominal_step = 0.0000012F;
    config.circle_yaw.kd_per_nominal_step = 0.000030F;
    config.circle_yaw.integral_limit = 0.009F;
    config.circle_yaw.maximum_step_deg = 0.022F;
    config.circle_pitch.kp = 0.00027F;
    config.circle_pitch.ki_per_nominal_step = 0.0000018F;
    config.circle_pitch.kd_per_nominal_step = 0.000052F;
    config.circle_pitch.integral_limit = 0.014F;
    config.circle_pitch.maximum_step_deg = 0.028F;

    config.lost_scan_delay_ms = 500U;
    config.lost_scan_half_span_deg = 45.0F;
    config.lost_scan_rate_deg_per_second = 8.0F;
    return config;
}

bool Mode_Gimbal_VisionStab_Init(
    Mode_Gimbal_VisionStabContext *context,
    const Mode_Gimbal_VisionStabConfig *config)
{
    Mode_Gimbal_GimbalStabContext stabilization;

    if ((context == NULL) || !valid_vision_config(config) ||
        !Mode_Gimbal_GimbalStab_Init(
            &stabilization, &config->stabilization)) {
        return false;
    }

    context->config = *config;
    context->stabilization = stabilization;
    context->entry_pitch_deg = 0.0F;
    context->entry_yaw_deg = 0.0F;
    context->target_pitch_deg = 0.0F;
    context->target_yaw_deg = 0.0F;
    context->filtered_dx_pixels = 0.0F;
    context->filtered_dy_pixels = 0.0F;
    context->yaw_integral = 0.0F;
    context->pitch_integral = 0.0F;
    context->previous_yaw_error = 0.0F;
    context->previous_pitch_error = 0.0F;
    context->control_elapsed_seconds = 0.0F;
    context->scan_center_yaw_deg = 0.0F;
    context->scan_direction = 1;
    context->last_sequence = 0U;
    context->loss_started_at_ms = 0U;
    context->target_initialized = false;
    context->filter_initialized = false;
    context->sequence_seen = false;
    context->previous_error_valid = false;
    context->loss_started = false;
    context->initialized = true;
    return true;
}

bool Mode_Gimbal_VisionStab_Update(
    Mode_Gimbal_VisionStabContext *context,
    const Mode_Gimbal_VisionStabInput *input,
    Mode_Gimbal_VisionStabOutput *output)
{
    Mode_Gimbal_VisionStabContext next;
    Mode_Gimbal_GimbalStabInput stabilization_input;
    Mode_Gimbal_GimbalStabOutput stabilization_output;
    const Mode_Gimbal_VisionAxisTuning *yaw_tuning;
    const Mode_Gimbal_VisionAxisTuning *pitch_tuning;
    bool vision_active;
    bool scanning = false;

    vision_stop_output(output);
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized ||
        !Mode_Gimbal_Internal_ValidTime(&input->time)) {
        return false;
    }
    if (input->vision.available && input->vision.valid &&
        (!Mode_Gimbal_Internal_IsFinite(input->vision.dx_pixels) ||
            !Mode_Gimbal_Internal_IsFinite(input->vision.dy_pixels))) {
        return false;
    }

    next = *context;
    if (!next.target_initialized && input->feedback.encoder_feedback_valid &&
        input->feedback.imu_feedback_valid) {
        if (!Mode_Gimbal_Internal_IsFinite(input->feedback.imu_pitch_deg) ||
            !Mode_Gimbal_Internal_IsFinite(input->feedback.imu_yaw_deg)) {
            return false;
        }
        next.entry_pitch_deg = input->feedback.imu_pitch_deg;
        next.entry_yaw_deg = Mode_Gimbal_Internal_NormalizeAngle(
            input->feedback.imu_yaw_deg);
        next.target_pitch_deg = next.entry_pitch_deg;
        next.target_yaw_deg = next.entry_yaw_deg;
        next.target_initialized = true;
    }

    vision_active = input->vision.available && input->vision.valid &&
        (Mode_Gimbal_Internal_ElapsedMs(input->time.now_ms,
             input->vision.received_at_ms) <= next.config.input_timeout_ms);
    if (vision_active) {
        float raw_dx = Mode_Gimbal_Internal_Clamp(input->vision.dx_pixels,
            -next.config.maximum_dx_pixels, next.config.maximum_dx_pixels);
        float raw_dy = Mode_Gimbal_Internal_Clamp(input->vision.dy_pixels,
            -next.config.maximum_dy_pixels, next.config.maximum_dy_pixels);

        if (!next.sequence_seen ||
            (input->vision.sequence != next.last_sequence)) {
            if (!next.filter_initialized) {
                next.filtered_dx_pixels = raw_dx;
                next.filtered_dy_pixels = raw_dy;
                next.filter_initialized = true;
            }
            else {
                next.filtered_dx_pixels =
                    (next.config.input_filter_alpha * raw_dx) +
                    ((1.0F - next.config.input_filter_alpha) *
                        next.filtered_dx_pixels);
                next.filtered_dy_pixels =
                    (next.config.input_filter_alpha * raw_dy) +
                    ((1.0F - next.config.input_filter_alpha) *
                        next.filtered_dy_pixels);
            }
            next.last_sequence = input->vision.sequence;
            next.sequence_seen = true;
        }
        next.loss_started = false;
        next.control_elapsed_seconds += input->time.dt_seconds;
        if (next.target_initialized &&
            (next.control_elapsed_seconds >=
                next.config.nominal_control_period_seconds)) {
            float step_ratio = next.control_elapsed_seconds /
                next.config.nominal_control_period_seconds;
            float yaw_error = pixel_deadband(next.filtered_dx_pixels,
                next.config.yaw_deadband_pixels);
            float pitch_error = pixel_deadband(next.filtered_dy_pixels,
                next.config.pitch_deadband_pixels);
            float yaw_step;
            float pitch_step;

            yaw_tuning = input->vision.circle_target ?
                &next.config.circle_yaw : &next.config.normal_yaw;
            pitch_tuning = input->vision.circle_target ?
                &next.config.circle_pitch : &next.config.normal_pitch;
            if (!update_vision_axis(yaw_tuning, yaw_error,
                    next.config.yaw_integral_deadband_pixels,
                    next.config.center_integral_decay,
                    next.config.reversal_integral_decay, step_ratio,
                    next.previous_error_valid, next.previous_yaw_error,
                    &next.yaw_integral, &yaw_step) ||
                !update_vision_axis(pitch_tuning, pitch_error,
                    next.config.pitch_integral_deadband_pixels,
                    next.config.center_integral_decay,
                    next.config.reversal_integral_decay, step_ratio,
                    next.previous_error_valid, next.previous_pitch_error,
                    &next.pitch_integral, &pitch_step)) {
                return false;
            }
            next.target_yaw_deg = Mode_Gimbal_Internal_NormalizeAngle(
                next.target_yaw_deg +
                (next.config.yaw_pixel_polarity * yaw_step));
            next.target_pitch_deg +=
                next.config.pitch_pixel_polarity * pitch_step;
            if (!Mode_Gimbal_Internal_IsFinite(next.target_pitch_deg)) {
                return false;
            }
            next.previous_yaw_error = yaw_error;
            next.previous_pitch_error = pitch_error;
            next.previous_error_valid = true;
            next.control_elapsed_seconds = fmodf(
                next.control_elapsed_seconds,
                next.config.nominal_control_period_seconds);
        }
    }
    else {
        float decay_ratio = input->time.dt_seconds /
            next.config.nominal_control_period_seconds;

        next.control_elapsed_seconds = 0.0F;
        next.yaw_integral *= powf(
            next.config.center_integral_decay, decay_ratio);
        next.pitch_integral *= powf(
            next.config.center_integral_decay, decay_ratio);
        next.previous_error_valid = false;
        if (next.target_initialized) {
            if (!next.loss_started) {
                next.loss_started = true;
                next.loss_started_at_ms = input->time.now_ms;
                next.scan_center_yaw_deg = next.target_yaw_deg;
                next.scan_direction = 1;
            }
            if (Mode_Gimbal_Internal_ElapsedMs(input->time.now_ms,
                    next.loss_started_at_ms) >=
                next.config.lost_scan_delay_ms) {
                update_scan_target(&next, input->time.dt_seconds);
                scanning = true;
            }
        }
    }

    stabilization_input.time = input->time;
    stabilization_input.feedback = input->feedback;
    stabilization_input.target_valid = next.target_initialized;
    stabilization_input.target_pitch_deg = next.target_pitch_deg;
    stabilization_input.target_yaw_deg = next.target_yaw_deg;
    if (!Mode_Gimbal_GimbalStab_Update(&next.stabilization,
            &stabilization_input, &stabilization_output)) {
        return false;
    }

    output->current = stabilization_output.current;
    output->feedback_ready = stabilization_output.feedback_ready;
    output->vision_active = vision_active;
    output->scanning = scanning;
    output->filtered_dx_pixels = next.filtered_dx_pixels;
    output->filtered_dy_pixels = next.filtered_dy_pixels;
    if (next.target_initialized) {
        output->pitch_offset_deg =
            next.target_pitch_deg - next.entry_pitch_deg;
        output->yaw_offset_deg = Mode_Gimbal_Internal_AngleError(
            next.target_yaw_deg, next.entry_yaw_deg);
    }
    *context = next;
    return true;
}
