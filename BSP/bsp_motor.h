#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/**
 * @brief 电机运行方向枚举
 */
typedef enum {
    BSP_MOTOR_DIR_FORWARD,   // 正转
    BSP_MOTOR_DIR_BACKWARD,  // 反转
    BSP_MOTOR_DIR_STOP       // 停止/刹车
} Motor_Dir_t;

/**
 * @brief 电机 ID 枚举 (A 为 0, B 为 1)
 */
typedef enum {
    MOTOR_ID_A = 0,
    MOTOR_ID_B = 1
} Motor_ID_t;

/**
 * @brief 初始化电机相关引脚、STBY 使能及 PWM 外设
 */
void BSP_Motor_Init(void);

/**
 * @brief 设置电机的运行状态与转速
 * @param motor_id 电机通道 (MOTOR_ID_A 或 MOTOR_ID_B)
 * @param dir 电机方向
 * @param pwm_duty PWM 占空比 (通常范围为 0~定时器自动重装载值)
 */
void BSP_Motor_SetState(Motor_ID_t motor_id, Motor_Dir_t dir, uint32_t pwm_duty);

/**
 * @brief 电机驱动待机/唤醒控制
 * @param sleep true为进入待机不上电，false为唤醒
 */
void BSP_Motor_Sleep(bool sleep);

#endif /* BSP_MOTOR_H */
