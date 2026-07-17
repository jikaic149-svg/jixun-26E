#ifndef MODE_GIMBAL_INTERNAL_H
#define MODE_GIMBAL_INTERNAL_H

#include "mode_gimbal.h"

bool Mode_Gimbal_Internal_IsFinite(float value);
bool Mode_Gimbal_Internal_ValidTime(const Mode_Gimbal_TimeInput *time);
float Mode_Gimbal_Internal_Clamp(float value, float minimum, float maximum);
float Mode_Gimbal_Internal_NormalizeAngle(float angle_deg);
float Mode_Gimbal_Internal_AngleError(
    float target_deg, float measured_deg);
uint32_t Mode_Gimbal_Internal_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
void Mode_Gimbal_Internal_StopIntent(Mode_Gimbal_CurrentIntent *intent);
bool Mode_Gimbal_Internal_ValidAngleConfig(
    const Mode_Gimbal_AngleLoopConfig *config);
bool Mode_Gimbal_Internal_ValidGimbalConfig(
    const Mode_Gimbal_GimbalStabConfig *config);

#endif
