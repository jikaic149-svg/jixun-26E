#include "mode_gimbal_internal.h"

#include <math.h>
#include <stddef.h>

static void stab_calib_stop_output(Mode_Gimbal_StabCalibOutput *output)
{
    if (output == NULL) {
        return;
    }
    Mode_Gimbal_Internal_StopIntent(&output->current);
    output->status = MODE_GIMBAL_STAB_CALIB_WAITING_FEEDBACK;
    output->locked_pitch_deg = 0.0F;
    output->locked_yaw_deg = 0.0F;
}

static bool valid_stab_calib_config(
    const Mode_Gimbal_StabCalibConfig *config)
{
    return (config != NULL) && (config->feedback_timeout_ms > 0U) &&
        Mode_Gimbal_Internal_ValidAngleConfig(&config->pitch_loop) &&
        Mode_Gimbal_Internal_ValidAngleConfig(&config->yaw_loop);
}

Mode_Gimbal_StabCalibConfig Mode_Gimbal_StabCalib_DefaultConfig(void)
{
    Mode_Gimbal_StabCalibConfig config;

    config.pitch_loop = Mode_Gimbal_AngleLoop_DefaultPitchConfig();
    config.yaw_loop = Mode_Gimbal_AngleLoop_DefaultYawConfig();
    config.feedback_timeout_ms = 100U;
    return config;
}

bool Mode_Gimbal_StabCalib_Init(
    Mode_Gimbal_StabCalibContext *context,
    const Mode_Gimbal_StabCalibConfig *config)
{
    Mode_Gimbal_AngleLoopContext pitch_loop;
    Mode_Gimbal_AngleLoopContext yaw_loop;

    if ((context == NULL) || !valid_stab_calib_config(config) ||
        !Mode_Gimbal_AngleLoop_Init(&pitch_loop, &config->pitch_loop) ||
        !Mode_Gimbal_AngleLoop_Init(&yaw_loop, &config->yaw_loop)) {
        return false;
    }

    context->config = *config;
    context->pitch_loop = pitch_loop;
    context->yaw_loop = yaw_loop;
    context->waiting_started_at_ms = 0U;
    context->locked_pitch_deg = 0.0F;
    context->locked_yaw_deg = 0.0F;
    context->waiting_started = false;
    context->target_locked = false;
    context->timed_out = false;
    context->initialized = true;
    return true;
}

bool Mode_Gimbal_StabCalib_Update(
    Mode_Gimbal_StabCalibContext *context,
    const Mode_Gimbal_StabCalibInput *input,
    Mode_Gimbal_StabCalibOutput *output)
{
    Mode_Gimbal_StabCalibContext next;
    Mode_Gimbal_AngleLoopInput loop_input;
    Mode_Gimbal_AngleLoopOutput pitch_output;
    Mode_Gimbal_AngleLoopOutput yaw_output;

    stab_calib_stop_output(output);
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized ||
        !Mode_Gimbal_Internal_ValidTime(&input->time)) {
        return false;
    }
    next = *context;
    output->locked_pitch_deg = next.locked_pitch_deg;
    output->locked_yaw_deg = next.locked_yaw_deg;

    if (next.timed_out) {
        output->status = MODE_GIMBAL_STAB_CALIB_FEEDBACK_TIMEOUT;
        return true;
    }
    if (!input->encoder_feedback_valid) {
        if (next.target_locked) {
            output->status = MODE_GIMBAL_STAB_CALIB_FEEDBACK_LOST;
            if (!Mode_Gimbal_AngleLoop_Init(
                    &next.pitch_loop, &next.config.pitch_loop) ||
                !Mode_Gimbal_AngleLoop_Init(
                    &next.yaw_loop, &next.config.yaw_loop)) {
                return false;
            }
        }
        else {
            if (!next.waiting_started) {
                next.waiting_started = true;
                next.waiting_started_at_ms = input->time.now_ms;
            }
            if (Mode_Gimbal_Internal_ElapsedMs(input->time.now_ms,
                    next.waiting_started_at_ms) >=
                next.config.feedback_timeout_ms) {
                next.timed_out = true;
                output->status =
                    MODE_GIMBAL_STAB_CALIB_FEEDBACK_TIMEOUT;
            }
            else {
                output->status =
                    MODE_GIMBAL_STAB_CALIB_WAITING_FEEDBACK;
            }
        }
        *context = next;
        return true;
    }
    if (!Mode_Gimbal_Internal_IsFinite(input->pitch_encoder_deg) ||
        !Mode_Gimbal_Internal_IsFinite(input->yaw_encoder_deg) ||
        !Mode_Gimbal_Internal_IsFinite(
            input->pitch_motor_speed_deg_per_second) ||
        !Mode_Gimbal_Internal_IsFinite(
            input->yaw_motor_speed_deg_per_second)) {
        return false;
    }

    if (!next.target_locked) {
        next.locked_pitch_deg = input->pitch_encoder_deg;
        next.locked_yaw_deg = input->yaw_encoder_deg;
        next.target_locked = true;
        if (!Mode_Gimbal_AngleLoop_Init(
                &next.pitch_loop, &next.config.pitch_loop) ||
            !Mode_Gimbal_AngleLoop_Init(
                &next.yaw_loop, &next.config.yaw_loop)) {
            return false;
        }
    }

    loop_input.dt_seconds = input->time.dt_seconds;
    loop_input.target_angle_deg = next.locked_pitch_deg;
    loop_input.measured_angle_deg = input->pitch_encoder_deg;
    loop_input.measured_speed_deg_per_second =
        input->pitch_motor_speed_deg_per_second;
    if (!Mode_Gimbal_AngleLoop_Update(
            &next.pitch_loop, &loop_input, &pitch_output)) {
        return false;
    }
    loop_input.target_angle_deg = next.locked_yaw_deg;
    loop_input.measured_angle_deg = input->yaw_encoder_deg;
    loop_input.measured_speed_deg_per_second =
        input->yaw_motor_speed_deg_per_second;
    if (!Mode_Gimbal_AngleLoop_Update(
            &next.yaw_loop, &loop_input, &yaw_output)) {
        return false;
    }

    output->current.pitch_current = pitch_output.current_intent;
    output->current.yaw_current = yaw_output.current_intent;
    output->current.current_requested = true;
    output->status = MODE_GIMBAL_STAB_CALIB_ENCODERS_LOCKED;
    output->locked_pitch_deg = next.locked_pitch_deg;
    output->locked_yaw_deg = next.locked_yaw_deg;
    *context = next;
    return true;
}

static void gimbal_stop_output(Mode_Gimbal_GimbalStabOutput *output)
{
    if (output == NULL) {
        return;
    }
    Mode_Gimbal_Internal_StopIntent(&output->current);
    output->feedback_ready = false;
    output->target_pitch_deg = 0.0F;
    output->target_yaw_deg = 0.0F;
    output->pitch_error_deg = 0.0F;
    output->yaw_encoder_target_deg = 0.0F;
}

static bool valid_full_feedback(const Mode_Gimbal_FeedbackInput *feedback)
{
    return (feedback != NULL) &&
        Mode_Gimbal_Internal_IsFinite(feedback->pitch_encoder_deg) &&
        Mode_Gimbal_Internal_IsFinite(feedback->yaw_encoder_deg) &&
        Mode_Gimbal_Internal_IsFinite(
            feedback->pitch_motor_speed_deg_per_second) &&
        Mode_Gimbal_Internal_IsFinite(
            feedback->yaw_motor_speed_deg_per_second) &&
        Mode_Gimbal_Internal_IsFinite(feedback->measured_pitch_current) &&
        Mode_Gimbal_Internal_IsFinite(feedback->imu_pitch_deg) &&
        Mode_Gimbal_Internal_IsFinite(feedback->imu_yaw_deg) &&
        Mode_Gimbal_Internal_IsFinite(
            feedback->imu_pitch_rate_deg_per_second) &&
        Mode_Gimbal_Internal_IsFinite(
            feedback->imu_yaw_rate_deg_per_second);
}

Mode_Gimbal_GimbalStabConfig Mode_Gimbal_GimbalStab_DefaultConfig(void)
{
    Mode_Gimbal_GimbalStabConfig config;

    config.pitch_kp = 0.030F;
    config.pitch_ki = 0.0F;
    config.pitch_kd_speed = 0.0055F;
    config.pitch_polarity = 1.0F;
    config.pitch_deadband_deg = 0.3F;
    config.pitch_maximum_error_deg = 16.0F;
    config.pitch_integral_limit = 0.0F;
    config.pitch_output_limit = 0.75F;
    config.pitch_speed_filter_old_weight = 0.3F;
    config.yaw_encoder_from_imu_polarity = -1.0F;
    config.yaw_loop = Mode_Gimbal_AngleLoop_DefaultYawConfig();
    return config;
}

bool Mode_Gimbal_GimbalStab_Init(
    Mode_Gimbal_GimbalStabContext *context,
    const Mode_Gimbal_GimbalStabConfig *config)
{
    Mode_Gimbal_AngleLoopContext yaw_loop;

    if ((context == NULL) ||
        !Mode_Gimbal_Internal_ValidGimbalConfig(config) ||
        !Mode_Gimbal_AngleLoop_Init(&yaw_loop, &config->yaw_loop)) {
        return false;
    }

    context->config = *config;
    context->yaw_loop = yaw_loop;
    context->target_pitch_deg = 0.0F;
    context->target_yaw_deg = 0.0F;
    context->pitch_integral_current = 0.0F;
    context->filtered_pitch_rate_deg_per_second = 0.0F;
    context->target_initialized = false;
    context->initialized = true;
    return true;
}

bool Mode_Gimbal_GimbalStab_Update(
    Mode_Gimbal_GimbalStabContext *context,
    const Mode_Gimbal_GimbalStabInput *input,
    Mode_Gimbal_GimbalStabOutput *output)
{
    Mode_Gimbal_GimbalStabContext next;
    Mode_Gimbal_AngleLoopInput yaw_input;
    Mode_Gimbal_AngleLoopOutput yaw_output;
    float pitch_error;
    float pitch_current;
    float yaw_imu_error;

    gimbal_stop_output(output);
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized ||
        !Mode_Gimbal_Internal_ValidTime(&input->time)) {
        return false;
    }
    if (!input->feedback.encoder_feedback_valid ||
        !input->feedback.imu_feedback_valid) {
        return true;
    }
    if (!valid_full_feedback(&input->feedback) ||
        (input->target_valid &&
            (!Mode_Gimbal_Internal_IsFinite(input->target_pitch_deg) ||
                !Mode_Gimbal_Internal_IsFinite(input->target_yaw_deg)))) {
        return false;
    }

    next = *context;
    if (!next.target_initialized) {
        next.target_pitch_deg = input->feedback.imu_pitch_deg;
        next.target_yaw_deg = Mode_Gimbal_Internal_NormalizeAngle(
            input->feedback.imu_yaw_deg);
        next.target_initialized = true;
    }
    if (input->target_valid) {
        next.target_pitch_deg = input->target_pitch_deg;
        next.target_yaw_deg = Mode_Gimbal_Internal_NormalizeAngle(
            input->target_yaw_deg);
    }

    pitch_error = (next.target_pitch_deg - input->feedback.imu_pitch_deg) *
        next.config.pitch_polarity;
    pitch_error = Mode_Gimbal_Internal_Clamp(pitch_error,
        -next.config.pitch_maximum_error_deg,
        next.config.pitch_maximum_error_deg);
    if (fabsf(pitch_error) <= next.config.pitch_deadband_deg) {
        pitch_error = 0.0F;
    }
    next.pitch_integral_current += next.config.pitch_ki * pitch_error *
        input->time.dt_seconds;
    next.pitch_integral_current = Mode_Gimbal_Internal_Clamp(
        next.pitch_integral_current, -next.config.pitch_integral_limit,
        next.config.pitch_integral_limit);
    next.filtered_pitch_rate_deg_per_second =
        (next.config.pitch_speed_filter_old_weight *
            next.filtered_pitch_rate_deg_per_second) +
        ((1.0F - next.config.pitch_speed_filter_old_weight) *
            input->feedback.imu_pitch_rate_deg_per_second);
    pitch_current = (next.config.pitch_kp * pitch_error) +
        next.pitch_integral_current -
        (next.config.pitch_kd_speed *
            next.filtered_pitch_rate_deg_per_second);
    if (!Mode_Gimbal_Internal_IsFinite(pitch_current)) {
        return false;
    }
    pitch_current = Mode_Gimbal_Internal_Clamp(pitch_current,
        -next.config.pitch_output_limit, next.config.pitch_output_limit);

    yaw_imu_error = Mode_Gimbal_Internal_AngleError(
        next.target_yaw_deg, input->feedback.imu_yaw_deg);
    yaw_input.dt_seconds = input->time.dt_seconds;
    yaw_input.target_angle_deg = Mode_Gimbal_Internal_NormalizeAngle(
        input->feedback.yaw_encoder_deg +
        (next.config.yaw_encoder_from_imu_polarity * yaw_imu_error));
    yaw_input.measured_angle_deg = input->feedback.yaw_encoder_deg;
    yaw_input.measured_speed_deg_per_second =
        input->feedback.yaw_motor_speed_deg_per_second;
    if (!Mode_Gimbal_AngleLoop_Update(
            &next.yaw_loop, &yaw_input, &yaw_output)) {
        return false;
    }

    output->current.pitch_current = pitch_current;
    output->current.yaw_current = yaw_output.current_intent;
    output->current.current_requested = true;
    output->feedback_ready = true;
    output->target_pitch_deg = next.target_pitch_deg;
    output->target_yaw_deg = next.target_yaw_deg;
    output->pitch_error_deg = pitch_error;
    output->yaw_encoder_target_deg = yaw_output.effective_target_deg;
    *context = next;
    return true;
}
