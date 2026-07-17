#include "mode_car_internal.h"

#include <math.h>
#include <stddef.h>

bool Mode_Car_Internal_IsFinite(float value)
{
    return isfinite(value) != 0;
}

bool Mode_Car_Internal_ValidTime(const Mode_Car_TimeInput *time)
{
    return (time != NULL) &&
        Mode_Car_Internal_IsFinite(time->dt_seconds) &&
        (time->dt_seconds > 0.0F);
}

void Mode_Car_Internal_StopOutput(
    Mode_Car_Output *output,
    Mode_Car_Completion completion)
{
    if (output == NULL) {
        return;
    }
    output->left_drive = 0.0F;
    output->right_drive = 0.0F;
    output->motion_requested = false;
    output->completion = completion;
}

bool Mode_Car_Internal_DriveOutput(
    float left_drive,
    float right_drive,
    Mode_Car_Output *output)
{
    if ((output == NULL) ||
        !Mode_Car_Internal_IsFinite(left_drive) ||
        !Mode_Car_Internal_IsFinite(right_drive)) {
        Mode_Car_Internal_StopOutput(
            output, MODE_CAR_COMPLETION_NONE);
        return false;
    }

    output->left_drive = left_drive;
    output->right_drive = right_drive;
    output->motion_requested =
        (left_drive != 0.0F) || (right_drive != 0.0F);
    output->completion = MODE_CAR_COMPLETION_NONE;
    return true;
}

bool Mode_Car_Internal_InitPID(
    Algorithm_Control_PID *pid,
    float kp,
    float ki,
    float kd,
    float integral_output_limit,
    float output_limit)
{
    Algorithm_Control_PIDConfig config;
    float raw_integral_limit = 0.0F;

    if (!Mode_Car_Internal_IsFinite(kp) ||
        !Mode_Car_Internal_IsFinite(ki) ||
        !Mode_Car_Internal_IsFinite(kd) ||
        !Mode_Car_Internal_IsFinite(integral_output_limit) ||
        !Mode_Car_Internal_IsFinite(output_limit) ||
        (integral_output_limit < 0.0F) || (output_limit < 0.0F)) {
        return false;
    }

    if (ki != 0.0F) {
        raw_integral_limit = integral_output_limit / fabsf(ki);
        if (!Mode_Car_Internal_IsFinite(raw_integral_limit)) {
            return false;
        }
    }

    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.integral_min = -raw_integral_limit;
    config.integral_max = raw_integral_limit;
    config.output_min = -output_limit;
    config.output_max = output_limit;
    config.integral_separation = 0.0F;
    config.derivative_mode = ALGORITHM_CONTROL_DERIVATIVE_ON_ERROR;
    return Algorithm_Control_PID_Init(pid, &config);
}
