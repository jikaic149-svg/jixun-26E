#include "mode_car_internal.h"

#include <stddef.h>

Mode_Car_SpinConfig Mode_Car_Spin_DefaultConfig(void)
{
    const Mode_Car_SpinConfig config = {
        .maximum_drive = 1.0F,
    };
    return config;
}

bool Mode_Car_Spin_Init(
    Mode_Car_SpinContext *context,
    const Mode_Car_SpinConfig *config)
{
    if (context != NULL) {
        *context = (Mode_Car_SpinContext){0};
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->maximum_drive) ||
        (config->maximum_drive <= 0.0F)) {
        return false;
    }

    context->config = *config;
    context->initialized = true;
    return true;
}

bool Mode_Car_Spin_Update(
    const Mode_Car_SpinContext *context,
    const Mode_Car_SpinInput *input,
    Mode_Car_Output *output)
{
    float spin_drive;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) || !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(input->spin_drive) ||
        !Algorithm_Control_Clamp(input->spin_drive,
            -context->config.maximum_drive,
            context->config.maximum_drive, &spin_drive)) {
        return false;
    }

    return Mode_Car_Internal_DriveOutput(
        -spin_drive, spin_drive, output);
}

Mode_Car_SpinRateConfig Mode_Car_SpinRate_DefaultConfig(void)
{
    const Mode_Car_SpinRateConfig config = {
        .kp = 1.5F,
        .ki = 0.5F,
        .integral_output_limit = 60.0F,
        .maximum_correction_drive = 100.0F,
        .feedforward_drive_per_dps = 2.0F,
        .maximum_drive = 220.0F,
    };
    return config;
}

bool Mode_Car_SpinRate_Init(
    Mode_Car_SpinRateContext *context,
    const Mode_Car_SpinRateConfig *config)
{
    Algorithm_Control_PID rate_pid;

    if (context != NULL) {
        *context = (Mode_Car_SpinRateContext){0};
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->feedforward_drive_per_dps) ||
        !Mode_Car_Internal_IsFinite(config->maximum_drive) ||
        (config->feedforward_drive_per_dps < 0.0F) ||
        (config->maximum_drive <= 0.0F) ||
        !Mode_Car_Internal_InitPID(&rate_pid,
            config->kp, config->ki, 0.0F,
            config->integral_output_limit,
            config->maximum_correction_drive)) {
        return false;
    }

    context->config = *config;
    context->rate_pid = rate_pid;
    context->initialized = true;
    return true;
}

void Mode_Car_SpinRate_Reset(Mode_Car_SpinRateContext *context)
{
    if ((context == NULL) || !context->initialized) {
        return;
    }
    Algorithm_Control_PID_Reset(&context->rate_pid);
}

bool Mode_Car_SpinRate_Update(
    Mode_Car_SpinRateContext *context,
    const Mode_Car_SpinRateInput *input,
    Mode_Car_Output *output)
{
    Algorithm_Control_PID next_pid;
    float correction;
    float spin_drive;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) || !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(
            input->target_yaw_rate_deg_per_second) ||
        !Mode_Car_Internal_IsFinite(
            input->measured_yaw_rate_deg_per_second)) {
        return false;
    }

    next_pid = context->rate_pid;
    if (!Algorithm_Control_PID_Update(&next_pid,
            input->target_yaw_rate_deg_per_second,
            input->measured_yaw_rate_deg_per_second,
            input->time.dt_seconds, &correction) ||
        !Algorithm_Control_Clamp(
            context->config.feedforward_drive_per_dps *
                input->target_yaw_rate_deg_per_second + correction,
            -context->config.maximum_drive,
            context->config.maximum_drive, &spin_drive) ||
        !Mode_Car_Internal_DriveOutput(
            -spin_drive, spin_drive, output)) {
        return false;
    }

    context->rate_pid = next_pid;
    return true;
}
