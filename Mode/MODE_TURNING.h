#ifndef __MODE_TURNING_H
#define __MODE_TURNING_H

#include <stdint.h>

void    MODE_Turning_Init(void);

/**
 * @brief 原地转向执行（IMU 航向闭环至目标角度）
 * @param target_yaw  目标航向角 (度)
 * @param current_yaw 当前航向角 (度)
 */
void    MODE_Turning_Exec(float target_yaw, float current_yaw);

/** @brief 是否已对准目标角度 */
uint8_t MODE_Turning_IsDone(void);

#endif /* __MODE_TURNING_H */
