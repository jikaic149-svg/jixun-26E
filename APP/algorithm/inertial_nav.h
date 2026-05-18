#ifndef APP_INERTIAL_NAV_H
#define APP_INERTIAL_NAV_H

#include <stdint.h>

#define INS_AX_FILTER_SIZE  5   /* 滑动窗口均值滤波点数 */

typedef struct {
    float speed_x;     // X轴积分速度 (mm/s)
    float mileage_x;   // X轴积分里程 (mm)
    
    /* 开机静止零飘自校准状态 */
    float ax_offset;
    uint16_t calib_count;

    /* ZUPT 零速检测 */
    uint16_t zupt_counter;

    /* 滑动窗口均值滤波 */
    float ax_filter_buf[INS_AX_FILTER_SIZE];
    uint8_t filter_idx;
    uint8_t filter_filled;  /* 缓冲区是否已填满 */
} INS_Instance;

void INS_Init(INS_Instance *inst);

/**
 * @brief  更新惯性导航测速系统
 * @param  inst      惯导实体
 * @param  ax_mps2   X轴当前加速度 (m/s^2)
 * @param  yaw_rate  当前航向角变化率 (°/s)，用于转弯里程补偿
 * @param  dt_s      调用间隔差值，例如 10ms 调度任务就是 0.01f 秒
 */
void INS_UpdateSpeedMileage(INS_Instance *inst, float ax_mps2, float yaw_rate, float dt_s);

#endif /* APP_INERTIAL_NAV_H */

