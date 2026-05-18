#include "bsp_motor.h"
#include "bsp_pwm.h"

void BSP_Motor_Init(void)
{
    // 使能 TB6612 的 STBY 引脚（高电平有效）
    DL_GPIO_setPins(STBY_PORT, STBY_PIN_STBY_PIN);
    
    // 初始化并启动内部 PWM 定时器
    BSP_PWM_Init();
    
    // 默认上电状态：驱动芯片进入待机（STBY 拉低，彻底不上电）
    BSP_Motor_Sleep(true);
    
    // 默认底层状态为停止
    BSP_Motor_SetState(MOTOR_ID_A, BSP_MOTOR_DIR_STOP, 0);
    BSP_Motor_SetState(MOTOR_ID_B, BSP_MOTOR_DIR_STOP, 0);
}

void BSP_Motor_SetState(Motor_ID_t motor_id, Motor_Dir_t dir, uint32_t pwm_duty)
{
    if (motor_id == MOTOR_ID_A) {
        // 配置 Motor A (右轮) 逻辑控制引脚：AIN1, AIN2
        if (dir == BSP_MOTOR_DIR_FORWARD) {
            DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_0_PIN);
            DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_1_PIN);
        } else if (dir == BSP_MOTOR_DIR_BACKWARD) {
            DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_0_PIN);
            DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_1_PIN);
        } else { // BSP_MOTOR_DIR_STOP
            DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_0_PIN);
            DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_1_PIN);
        }
        // 更新 Motor A PWM (PWMA)
        BSP_PWM_SetDutyL(pwm_duty);
    } 
    else if (motor_id == MOTOR_ID_B) {
        // 配置 Motor B (左轮) 逻辑控制引脚：BIN1, BIN2
        if (dir == BSP_MOTOR_DIR_FORWARD) {
            DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_2_PIN);
            DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_3_PIN);
        } else if (dir == BSP_MOTOR_DIR_BACKWARD) {
            DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_2_PIN);
            DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_3_PIN);
        } else { // BSP_MOTOR_DIR_STOP
            DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_2_PIN);
            DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_3_PIN);
        }
        // 更新 Motor B PWM (PWMB)
        BSP_PWM_SetDutyR(pwm_duty);
    }
}

void BSP_Motor_Sleep(bool sleep)
{
    if (sleep) {
        DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN); // STBY=L 待机
    } else {
        DL_GPIO_setPins(STBY_PORT, STBY_PIN_STBY_PIN);   // STBY=H 工作
    }
}
