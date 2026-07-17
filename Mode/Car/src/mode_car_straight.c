#include "mode_car_internal.h"

#include <math.h>
#include <stddef.h>

Mode_Car_StraightConfig Mode_Car_Straight_DefaultConfig(void)
{
    const Mode_Car_StraightConfig config = {
        .kp = 1.5F,
        .ki = 0.0F,
        .kd = 0.2F,
        .gain_reference_seconds = 0.005F,
        .integral_output_limit = 300.0F,
        .maximum_correction = 200.0F,
        .correction_ratio = 0.5F,
    };
    return config;
}

bool Mode_Car_Straight_Init(
    Mode_Car_StraightContext *context,
    const Mode_Car_StraightConfig *config)
{
    Algorithm_Control_PID pid;
    float effective_ki;
    float effective_kd;

    if (context != NULL) {
        context->initialized = false;
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->gain_reference_seconds) ||
        !Mode_Car_Internal_IsFinite(config->correction_ratio) ||
        (config->gain_reference_seconds <= 0.0F) ||
        (config->correction_ratio < 0.0F) ||
        !Mode_Car_Internal_IsFinite(
            effective_ki = config->ki / config->gain_reference_seconds) ||
        !Mode_Car_Internal_IsFinite(
            effective_kd = config->kd * config->gain_reference_seconds) ||
        !Mode_Car_Internal_InitPID(&pid,
            config->kp, effective_ki, effective_kd,
            config->integral_output_limit, config->maximum_correction)) {
        return false;
    }

    context->config = *config;
    context->heading_pid = pid;
    context->initialized = true;
    return true;
}

bool Mode_Car_Straight_Update(
    Mode_Car_StraightContext *context,
    const Mode_Car_StraightInput *input,
    Mode_Car_Output *output)
{
    Algorithm_Control_PID next_pid;
    float heading_error;
    float correction;
    float proportional_limit;
    float left_drive;
    float right_drive;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) ||
        !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(input->base_drive) ||
        !Mode_Car_Internal_IsFinite(input->target_heading_deg) ||
        !Mode_Car_Internal_IsFinite(input->measured_heading_deg) ||
        !Algorithm_Control_AngleErrorDeg(input->target_heading_deg,
            input->measured_heading_deg, &heading_error)) {
        return false;
    }

    next_pid = context->heading_pid;
    if (!Algorithm_Control_PID_Update(&next_pid,
            heading_error, 0.0F, input->time.dt_seconds, &correction)) {
        return false;
    }

    proportional_limit =
        fabsf(input->base_drive) * context->config.correction_ratio;
    if (!Mode_Car_Internal_IsFinite(proportional_limit) ||
        !Algorithm_Control_Clamp(correction, -proportional_limit,
            proportional_limit, &correction)) {
        return false;
    }

    left_drive = input->base_drive - correction;
    right_drive = input->base_drive + correction;
    if (!Mode_Car_Internal_DriveOutput(
            left_drive, right_drive, output)) {
        return false;
    }

    context->heading_pid = next_pid;
    return true;
}
