#include "mode_car_internal.h"

#include <math.h>
#include <stddef.h>

Mode_Car_TrackingConfig Mode_Car_Tracking_DefaultConfig(void)
{
    const Mode_Car_TrackingConfig config = {
        .kp = 1.2F,
        .ki = 0.0F,
        .kd = 0.03F,
        .integral_output_limit = 0.5F,
        .maximum_correction = 0.8F,
        .maximum_drive = 1.0F,
        .error_deadband = 0.0F,
        .stop_on_intersection = false,
    };
    return config;
}

bool Mode_Car_Tracking_Init(
    Mode_Car_TrackingContext *context,
    const Mode_Car_TrackingConfig *config)
{
    Algorithm_Control_PID pid;

    if (context != NULL) {
        *context = (Mode_Car_TrackingContext){0};
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->maximum_drive) ||
        !Mode_Car_Internal_IsFinite(config->error_deadband) ||
        (config->maximum_drive <= 0.0F) ||
        (config->error_deadband < 0.0F) ||
        !Mode_Car_Internal_InitPID(&pid,
            config->kp, config->ki, config->kd,
            config->integral_output_limit,
            config->maximum_correction)) {
        return false;
    }

    context->config = *config;
    context->line_pid = pid;
    context->initialized = true;
    return true;
}

bool Mode_Car_Tracking_Update(
    Mode_Car_TrackingContext *context,
    const Mode_Car_TrackingInput *input,
    Mode_Car_Output *output)
{
    Algorithm_Control_PID next_pid;
    float error;
    float correction;
    float left_drive;
    float right_drive;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) || !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(input->base_drive) ||
        !Mode_Car_Internal_IsFinite(input->line_error)) {
        return false;
    }

    if (!input->line_valid || input->line_lost ||
        (input->intersection && context->config.stop_on_intersection)) {
        Algorithm_Control_PID_Reset(&context->line_pid);
        return true;
    }

    if (!Algorithm_Control_Deadband(
            input->line_error, context->config.error_deadband, &error)) {
        return false;
    }
    next_pid = context->line_pid;
    /* Positive error means the line is right of the car, so the left wheel
     * must run faster and the right wheel slower. */
    if (!Algorithm_Control_PID_Update(&next_pid,
            error, 0.0F, input->time.dt_seconds, &correction) ||
        !Algorithm_Control_Clamp(input->base_drive + correction,
            -context->config.maximum_drive,
            context->config.maximum_drive, &left_drive) ||
        !Algorithm_Control_Clamp(input->base_drive - correction,
            -context->config.maximum_drive,
            context->config.maximum_drive, &right_drive) ||
        !Mode_Car_Internal_DriveOutput(left_drive, right_drive, output)) {
        return false;
    }

    context->line_pid = next_pid;
    return true;
}
