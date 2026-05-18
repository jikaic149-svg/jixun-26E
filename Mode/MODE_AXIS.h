#ifndef __MODE_AXIS_H
#define __MODE_AXIS_H

/**
 * @file  MODE_AXIS.h
 * @brief 坐标导航模块占位 (需要编码器里程计, 当前硬件不支持)
 * @note  当前项目无电机速度闭环和编码器反馈, 此模块暂不可用
 *        保留接口以备后续扩展
 */

#include <stdint.h>

void    MODE_Axis_Init(float target_x_mm, float target_y_mm);
void    MODE_Axis_Exec(void);
uint8_t MODE_Axis_IsDone(void);

#endif /* __MODE_AXIS_H */
