#ifndef __MODE_TRACKING_H
#define __MODE_TRACKING_H

#include <stdint.h>

void MODE_Tracking_Init(void);

/**
 * @brief 循迹模式执行（红外 PID 横向纠偏，恒定基速）
 * @param base_speed    基础前进速度 (mm/s)，恒定不变
 * @param current_error 循迹误差 (>0 线在左, <0 线在右, 999.0 丢线)
 */
void MODE_Tracking_Exec(float base_speed, float current_error);

#endif /* __MODE_TRACKING_H */
