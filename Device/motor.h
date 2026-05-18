/**
 * @file motor.h
 * @brief Device层：面向对象的电机驱动封装模型 (无需修改即可在不同单片机复用)
 */
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* 定义方向常量 (兼容大多数电机驱动如 TB6612) */
#define MOTOR_DIR_STOP     (0)
#define MOTOR_DIR_FORWARD  (1)
#define MOTOR_DIR_BACKWARD (-1)

/* 硬件接口层虚函数表 (HAL 映射钩子) 
 * 【说明】Device模块绝不 include <stm32.h> 或 <msp.h> 
 * 所有底层操作必须通过实例化时的注册映射强制分发给 BSP
 */
typedef struct {
    void (*SetDir)(int8_t dir);
    void (*SetPWM)(uint32_t duty);
} Motor_HW_API_t;

/* Motor 对象配置结构体 */
typedef struct {
    uint32_t max_duty;      // 占空比最大限制 (如 1000)
    uint32_t dead_zone;     // 占空比死区 (如 5，低于此值直接输出 STOP 停转)
    Motor_HW_API_t hw_api;  // 底层驱动隔离适配层
} MotorConfig_t;

/* Motor 对象句柄 (实例化自身) */
typedef struct {
    uint32_t max_duty;
    uint32_t dead_zone;
    Motor_HW_API_t hw_api;
} Motor_t;

/* 返回枚举 */
typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_NULL_PTR,
    MOTOR_ERR_HAL_MISSING
} MotorStatus_e;

/**
 * @brief 初始化电机实例
 * @param motor 传入待初始化的电机句柄实例
 * @param config 参数配置
 * @return 初始化状态
 */
MotorStatus_e Motor_Init(Motor_t* motor, const MotorConfig_t* config);

/**
 * @brief 输出开环控制信号给电机
 * @param motor 电机句柄实例
 * @param target_pwm 有符号占空比 (-max_duty ~ max_duty)。正为正转，负为反转。
 */
void Motor_SetDrive(Motor_t* motor, float target_pwm);

#endif /* DEVICE_MOTOR_H */
