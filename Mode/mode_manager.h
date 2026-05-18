/**
 * @file  mode_manager.h
 * @brief 小车行为状态机管理器（Application 层，零硬件依赖）
 *
 * 职责：
 *   1. 维护当前运行模式 (CarMode_e)
 *   2. 提供统一的模式切换 API（自动调用各模式的 Init）
 *   3. 在 10ms 调度任务中被 schedule.c 调用 Mode_Exec()
 *      内部自动分发到对应模式的 Exec 函数
 *
 * 数据流：
 *   传感器 → Broker 发布 → Mode_Exec 订阅 → 计算 → Broker 发布电机PWM → schedule 驱动
 */
#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdint.h>

/* ======================== 模式枚举 ======================== */
typedef enum {
    CAR_MODE_IDLE = 0,      /* 空闲：电机输出 0 */
    CAR_MODE_STRAIGHT,      /* 直线：IMU 航向纠偏 + 定 PWM 前进 */
    CAR_MODE_TRACKING,      /* 循迹：红外闭环 + 自适应变速 */
    CAR_MODE_TURNING,       /* 原地转向：IMU 航向闭环到目标角度 */
    CAR_MODE_CURVE,         /* 弧线行进：前进 + 渐进式航向修正 */
    CAR_MODE_BRAKE,         /* 制动：输出 0 + 等待超时 */
    CAR_MODE_DONE,          /* 完成态：永久停机 */
    CAR_MODE_TASK_TEST,     /* 任务：PID基础测试流水线 */
    CAR_MODE_MAX
} CarMode_e;

/**
 * @brief 初始化模式管理器（在 Schedule_Init 中调用一次）
 */
void Mode_Init(void);

/**
 * @brief 执行当前模式的周期逻辑（在 10ms 任务中调用）
 */
void Mode_Exec(void);

/**
 * @brief 获取当前运行模式
 */
CarMode_e Mode_GetMode(void);

/* ======================== 模式切换 API ======================== */

/** @brief 切入空闲模式 */
void Mode_SetIdle(void);

/**
 * @brief 切入直线模式
 * @param base_pwm   基础 PWM 占空比 (0~1000)
 * @param target_yaw 锁定的目标航向角 (度)
 */
void Mode_SetStraight(float base_pwm, float target_yaw);

/**
 * @brief 切入循迹模式
 * @param max_pwm 循迹最大 PWM 占空比 (弯道自适应降速)
 */
void Mode_SetTracking(float max_pwm);

/**
 * @brief 切入原地转向模式
 * @param target_yaw 目标航向角 (度)
 */
void Mode_SetTurning(float target_yaw);

/**
 * @brief 切入弧线行进模式
 * @param base_pwm   基础 PWM 占空比
 * @param target_yaw 目标航向角 (度)
 */
void Mode_SetCurve(float base_pwm, float target_yaw);

/** @brief 切入制动模式 */
void Mode_SetBrake(void);

/** @brief 切入完成态 */
void Mode_SetDone(void);

/** @brief 切入矩形循迹任务（task_id 仅占位，唯一实现的任务） */
void Mode_SetTask(uint8_t task_id);

/* ======================== 查询接口 ======================== */

/** @brief 转向模式是否已完成对准 (返回 1=完成) */
uint8_t Mode_IsTurningDone(void);

/** @brief 制动模式是否已停稳 (返回 1=完成) */
uint8_t Mode_IsBrakeDone(void);

#endif /* MODE_MANAGER_H */
