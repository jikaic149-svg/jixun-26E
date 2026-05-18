#ifndef __MODE_STRAIGHT_H
#define __MODE_STRAIGHT_H

#include <stdint.h>

void MODE_Straight_Init(void);

/**
 * @brief 直线行驶执行（IMU 航向纠偏 + 开环 PWM 前进）
 * @param base_pwm    基础 PWM 占空比 (0~1000)
 * @param target_yaw  目标航向角 (度)
 * @param current_yaw 当前航向角 (度，来自 JY61P)
 */
void MODE_Straight_Exec(float base_pwm, float target_yaw, float current_yaw);

#endif /* __MODE_STRAIGHT_H */
