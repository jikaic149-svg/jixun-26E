#ifndef MODE_CAR_INTERNAL_H
#define MODE_CAR_INTERNAL_H

#include "mode_car.h"

bool Mode_Car_Internal_IsFinite(float value);
bool Mode_Car_Internal_ValidTime(const Mode_Car_TimeInput *time);
void Mode_Car_Internal_StopOutput(
    Mode_Car_Output *output,
    Mode_Car_Completion completion);
bool Mode_Car_Internal_DriveOutput(
    float left_drive,
    float right_drive,
    Mode_Car_Output *output);
bool Mode_Car_Internal_InitPID(
    Algorithm_Control_PID *pid,
    float kp,
    float ki,
    float kd,
    float integral_output_limit,
    float output_limit);

#endif
