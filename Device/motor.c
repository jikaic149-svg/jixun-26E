/**
 * @file motor.c
 * @brief Device层：面向对象的电机驱动封装模型实现
 */
#include "motor.h"

MotorStatus_e Motor_Init(Motor_t* motor, const MotorConfig_t* config)
{
    if (motor == NULL || config == NULL) {
        return MOTOR_ERR_NULL_PTR;
    }
    
    // 强制检查是否严格实现了底层映射代理，防空指针异常
    if (config->hw_api.SetDir == NULL || config->hw_api.SetPWM == NULL) {
        return MOTOR_ERR_HAL_MISSING;
    }

    motor->max_duty = config->max_duty;
    motor->dead_zone = config->dead_zone;
    motor->hw_api = config->hw_api;

    return MOTOR_OK;
}

void Motor_SetDrive(Motor_t* motor, float target_pwm)
{
    if (motor == NULL) return;

    /* 死区屏蔽处理，滤除高频窄干扰 */
    if (target_pwm <= (float)motor->dead_zone && target_pwm >= -(float)motor->dead_zone) {
        motor->hw_api.SetDir(MOTOR_DIR_STOP);
        motor->hw_api.SetPWM(0);
        return;
    }

    /* 将带符号的驱动量拆分为 方向 + 绝对占空比 */
    int8_t dir;
    uint32_t abs_pwm;

    if (target_pwm > 0.0f) {
        dir = MOTOR_DIR_FORWARD;
        abs_pwm = (uint32_t)target_pwm;
    } else {
        dir = MOTOR_DIR_BACKWARD;
        abs_pwm = (uint32_t)(-target_pwm);
    }

    /* 满量程限幅保护 */
    if (abs_pwm > motor->max_duty) {
        abs_pwm = motor->max_duty;
    }

    /* 通过虚表调用真实的 BSP 动作 */
    motor->hw_api.SetDir(dir);
    motor->hw_api.SetPWM(abs_pwm);
}
