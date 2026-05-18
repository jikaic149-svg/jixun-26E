#ifndef __MODE_CURVE_H
#define __MODE_CURVE_H

#include <stdint.h>

void MODE_Curve_Init(void);

/**
 * @brief 弧线行进执行（前进 + 渐进式航向修正）
 * @param base_pwm    基础 PWM 占空比
 * @param target_yaw  目标航向角 (度)
 * @param current_yaw 当前航向角 (度)
 */
void MODE_Curve_Exec(float base_pwm, float target_yaw, float current_yaw);

#endif /* __MODE_CURVE_H */
