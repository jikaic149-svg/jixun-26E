#include "mode_car_internal.h"

#include <stddef.h>

Mode_Car_CurveConfig Mode_Car_Curve_DefaultConfig(void)
{
    const Mode_Car_CurveConfig config = {
        .outer_kp = 100.0F,
        .outer_ki = 0.0F,
        .outer_integral_output_limit = 160.0F,
        .maximum_heading_rate_deg_per_second = 160.0F,
        .outer_period_seconds = 0.020F,
        .inner_kp = 1.5F,
        .inner_ki = 0.1F,
        .inner_integral_output_limit = 100.0F,
        .maximum_drive_difference = 250.0F,
        .drive_difference_slew_per_second = 700.0F,
    };
    return config;
}

bool Mode_Car_Curve_Init(
    Mode_Car_CurveContext *context,
    const Mode_Car_CurveConfig *config)
{
    Algorithm_Control_PID heading_pid;
    Algorithm_Control_PID rate_pid;

    if (context != NULL) {
        context->initialized = false;
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->outer_period_seconds) ||
        !Mode_Car_Internal_IsFinite(
            config->drive_difference_slew_per_second) ||
        (config->outer_period_seconds <= 0.0F) ||
        (config->drive_difference_slew_per_second < 0.0F) ||
        !Mode_Car_Internal_InitPID(&heading_pid,
            config->outer_kp, config->outer_ki, 0.0F,
            config->outer_integral_output_limit,
            config->maximum_heading_rate_deg_per_second) ||
        !Mode_Car_Internal_InitPID(&rate_pid,
            config->inner_kp, config->inner_ki, 0.0F,
            config->inner_integral_output_limit,
            config->maximum_drive_difference)) {
        return false;
    }

    context->config = *config;
    context->heading_pid = heading_pid;
    context->heading_rate_pid = rate_pid;
    context->target_heading_rate_deg_per_second = 0.0F;
    context->drive_difference = 0.0F;
    context->outer_elapsed_seconds = 0.0F;
    context->outer_initialized = false;
    context->initialized = true;
    return true;
}

bool Mode_Car_Curve_Update(
    Mode_Car_CurveContext *context,
    const Mode_Car_CurveInput *input,
    Mode_Car_Output *output)
{
    Algorithm_Control_PID next_heading_pid;
    Algorithm_Control_PID next_rate_pid;
    float next_target_rate;
    float next_difference;
    float next_outer_elapsed;
    float heading_error;
    float raw_difference;
    float left_drive;
    float right_drive;
    bool next_outer_initialized;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) ||
        !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(input->base_drive) ||
        !Mode_Car_Internal_IsFinite(input->target_heading_deg) ||
        !Mode_Car_Internal_IsFinite(input->measured_heading_deg) ||
        !Mode_Car_Internal_IsFinite(
            input->measured_heading_rate_deg_per_second)) {
        return false;
    }

    next_heading_pid = context->heading_pid;
    next_rate_pid = context->heading_rate_pid;
    next_target_rate = context->target_heading_rate_deg_per_second;
    next_difference = context->drive_difference;
    next_outer_elapsed =
        context->outer_elapsed_seconds + input->time.dt_seconds;
    next_outer_initialized = context->outer_initialized;
    if (!Mode_Car_Internal_IsFinite(next_outer_elapsed)) {
        return false;
    }

    if (!next_outer_initialized ||
        (next_outer_elapsed >= context->config.outer_period_seconds)) {
        if (!Algorithm_Control_AngleErrorDeg(input->target_heading_deg,
                input->measured_heading_deg, &heading_error) ||
            !Algorithm_Control_PID_Update(&next_heading_pid,
                heading_error, 0.0F, next_outer_elapsed,
                &next_target_rate)) {
            return false;
        }
        next_outer_elapsed = 0.0F;
        next_outer_initialized = true;
    }

    if (!Algorithm_Control_PID_Update(&next_rate_pid,
            next_target_rate,
            input->measured_heading_rate_deg_per_second,
            input->time.dt_seconds,
            &raw_difference) ||
        !Algorithm_Control_Slew(next_difference, raw_difference,
            context->config.drive_difference_slew_per_second,
            input->time.dt_seconds, &next_difference)) {
        return false;
    }

    left_drive = input->base_drive - next_difference;
    right_drive = input->base_drive + next_difference;
    if (!Mode_Car_Internal_DriveOutput(
            left_drive, right_drive, output)) {
        return false;
    }

    context->heading_pid = next_heading_pid;
    context->heading_rate_pid = next_rate_pid;
    context->target_heading_rate_deg_per_second = next_target_rate;
    context->drive_difference = next_difference;
    context->outer_elapsed_seconds = next_outer_elapsed;
    context->outer_initialized = next_outer_initialized;
    return true;
}
