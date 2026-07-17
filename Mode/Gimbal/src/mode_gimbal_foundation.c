#include "mode_gimbal_internal.h"

#include <math.h>
#include <stddef.h>

bool Mode_Gimbal_Internal_IsFinite(float value)
{
    return isfinite(value) != 0;
}

bool Mode_Gimbal_Internal_ValidTime(const Mode_Gimbal_TimeInput *time)
{
    return (time != NULL) &&
        Mode_Gimbal_Internal_IsFinite(time->dt_seconds) &&
        (time->dt_seconds > 0.0F);
}

float Mode_Gimbal_Internal_Clamp(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

float Mode_Gimbal_Internal_NormalizeAngle(float angle_deg)
{
    float normalized = fmodf(angle_deg, 360.0F);

    if (normalized >= 180.0F) {
        normalized -= 360.0F;
    }
    else if (normalized < -180.0F) {
        normalized += 360.0F;
    }
    return normalized;
}

float Mode_Gimbal_Internal_AngleError(
    float target_deg, float measured_deg)
{
    return Mode_Gimbal_Internal_NormalizeAngle(target_deg - measured_deg);
}

uint32_t Mode_Gimbal_Internal_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

void Mode_Gimbal_Internal_StopIntent(Mode_Gimbal_CurrentIntent *intent)
{
    if (intent == NULL) {
        return;
    }
    intent->pitch_current = 0.0F;
    intent->yaw_current = 0.0F;
    intent->current_requested = false;
}

bool Mode_Gimbal_Internal_ValidAngleConfig(
    const Mode_Gimbal_AngleLoopConfig *config)
{
    if ((config == NULL) ||
        !Mode_Gimbal_Internal_IsFinite(config->kp) ||
        !Mode_Gimbal_Internal_IsFinite(config->ki) ||
        !Mode_Gimbal_Internal_IsFinite(config->kd_speed) ||
        !Mode_Gimbal_Internal_IsFinite(config->polarity) ||
        !Mode_Gimbal_Internal_IsFinite(config->deadband_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->maximum_error_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->integral_separation_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->integral_limit) ||
        !Mode_Gimbal_Internal_IsFinite(config->output_limit) ||
        !Mode_Gimbal_Internal_IsFinite(
            config->feedforward_threshold_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->positive_feedforward) ||
        !Mode_Gimbal_Internal_IsFinite(config->negative_feedforward) ||
        (config->polarity == 0.0F) || (config->deadband_deg < 0.0F) ||
        (config->maximum_error_deg <= 0.0F) ||
        (config->integral_separation_deg < 0.0F) ||
        (config->integral_limit < 0.0F) || (config->output_limit <= 0.0F) ||
        (config->feedforward_threshold_deg < 0.0F)) {
        return false;
    }
    if (config->target_limit_enabled &&
        (!Mode_Gimbal_Internal_IsFinite(config->target_minimum_deg) ||
            !Mode_Gimbal_Internal_IsFinite(config->target_maximum_deg) ||
            (config->target_minimum_deg > config->target_maximum_deg))) {
        return false;
    }
    return true;
}

bool Mode_Gimbal_Internal_ValidGimbalConfig(
    const Mode_Gimbal_GimbalStabConfig *config)
{
    return (config != NULL) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_kp) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_ki) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_kd_speed) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_polarity) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_deadband_deg) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_maximum_error_deg) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_integral_limit) &&
        Mode_Gimbal_Internal_IsFinite(config->pitch_output_limit) &&
        Mode_Gimbal_Internal_IsFinite(
            config->pitch_speed_filter_old_weight) &&
        Mode_Gimbal_Internal_IsFinite(
            config->yaw_encoder_from_imu_polarity) &&
        (config->pitch_polarity != 0.0F) &&
        (config->pitch_deadband_deg >= 0.0F) &&
        (config->pitch_maximum_error_deg > 0.0F) &&
        (config->pitch_integral_limit >= 0.0F) &&
        (config->pitch_output_limit > 0.0F) &&
        (config->pitch_speed_filter_old_weight >= 0.0F) &&
        (config->pitch_speed_filter_old_weight < 1.0F) &&
        (config->yaw_encoder_from_imu_polarity != 0.0F) &&
        Mode_Gimbal_Internal_ValidAngleConfig(&config->yaw_loop);
}

Mode_Gimbal_AngleLoopConfig
Mode_Gimbal_AngleLoop_DefaultPitchConfig(void)
{
    Mode_Gimbal_AngleLoopConfig config;

    config.kp = 0.038F;
    config.ki = 0.006F;
    config.kd_speed = 0.0045F;
    config.polarity = 1.0F;
    config.deadband_deg = 0.3F;
    config.maximum_error_deg = 14.0F;
    config.integral_separation_deg = 1.5F;
    config.integral_limit = 0.025F;
    config.output_limit = 0.75F;
    config.feedforward_threshold_deg = 0.8F;
    config.positive_feedforward = 0.0F;
    config.negative_feedforward = 0.0F;
    config.wrap_error = false;
    config.target_limit_enabled = true;
    config.target_minimum_deg = 19.0F;
    config.target_maximum_deg = 112.0F;
    return config;
}

Mode_Gimbal_AngleLoopConfig
Mode_Gimbal_AngleLoop_DefaultYawConfig(void)
{
    Mode_Gimbal_AngleLoopConfig config;

    config.kp = 0.032F;
    config.ki = 0.006F;
    config.kd_speed = 0.005F;
    config.polarity = 1.0F;
    config.deadband_deg = 0.35F;
    config.maximum_error_deg = 8.0F;
    config.integral_separation_deg = 2.0F;
    config.integral_limit = 0.015F;
    config.output_limit = 0.32F;
    config.feedforward_threshold_deg = 0.8F;
    config.positive_feedforward = 0.0F;
    config.negative_feedforward = 0.0F;
    config.wrap_error = true;
    config.target_limit_enabled = false;
    config.target_minimum_deg = 0.0F;
    config.target_maximum_deg = 0.0F;
    return config;
}

bool Mode_Gimbal_AngleLoop_Init(
    Mode_Gimbal_AngleLoopContext *context,
    const Mode_Gimbal_AngleLoopConfig *config)
{
    if ((context == NULL) ||
        !Mode_Gimbal_Internal_ValidAngleConfig(config)) {
        return false;
    }

    context->config = *config;
    context->integral_current = 0.0F;
    context->initialized = true;
    return true;
}

bool Mode_Gimbal_AngleLoop_Update(
    Mode_Gimbal_AngleLoopContext *context,
    const Mode_Gimbal_AngleLoopInput *input,
    Mode_Gimbal_AngleLoopOutput *output)
{
    Mode_Gimbal_AngleLoopContext next;
    float target;
    float error;
    float feedforward = 0.0F;
    float current;

    if (output != NULL) {
        output->current_intent = 0.0F;
        output->effective_target_deg = 0.0F;
        output->error_deg = 0.0F;
    }
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized ||
        !Mode_Gimbal_Internal_IsFinite(input->dt_seconds) ||
        (input->dt_seconds <= 0.0F) ||
        !Mode_Gimbal_Internal_IsFinite(input->target_angle_deg) ||
        !Mode_Gimbal_Internal_IsFinite(input->measured_angle_deg) ||
        !Mode_Gimbal_Internal_IsFinite(
            input->measured_speed_deg_per_second)) {
        return false;
    }

    next = *context;
    target = input->target_angle_deg;
    if (next.config.target_limit_enabled) {
        target = Mode_Gimbal_Internal_Clamp(target,
            next.config.target_minimum_deg, next.config.target_maximum_deg);
    }
    error = next.config.wrap_error ?
        Mode_Gimbal_Internal_AngleError(target, input->measured_angle_deg) :
        (target - input->measured_angle_deg);
    error *= next.config.polarity;
    error = Mode_Gimbal_Internal_Clamp(error,
        -next.config.maximum_error_deg, next.config.maximum_error_deg);
    if (fabsf(error) <= next.config.deadband_deg) {
        error = 0.0F;
    }

    if ((next.config.integral_separation_deg <= 0.0F) ||
        (fabsf(error) <= next.config.integral_separation_deg)) {
        next.integral_current +=
            next.config.ki * error * input->dt_seconds;
        next.integral_current = Mode_Gimbal_Internal_Clamp(
            next.integral_current, -next.config.integral_limit,
            next.config.integral_limit);
    }

    if (fabsf(error) >= next.config.feedforward_threshold_deg) {
        feedforward = (error > 0.0F) ? next.config.positive_feedforward :
            next.config.negative_feedforward;
    }
    current = (next.config.kp * error) + next.integral_current +
        feedforward -
        (next.config.kd_speed * input->measured_speed_deg_per_second);
    if (!Mode_Gimbal_Internal_IsFinite(current)) {
        return false;
    }
    current = Mode_Gimbal_Internal_Clamp(current,
        -next.config.output_limit, next.config.output_limit);

    output->current_intent = current;
    output->effective_target_deg = target;
    output->error_deg = error;
    *context = next;
    return true;
}
