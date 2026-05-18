/**
 * @file  MODE_STRAIGHT.c
 * @brief 直线行驶模式：固定 PWM 前进 + IMU 航向 PID 纠偏
 *        无速度闭环，输出为原始 PWM 占空比
 */
#include "MODE_STRAIGHT.h"
#include "../APP/broker/broker.h"
#include "../APP/algorithm/pid_controller.h"
#include "../APP/app_config.h"

static PIDInstance s_yaw_pid;

void MODE_Straight_Init(void)
{
    PID_Init_Config_s cfg = {
        .Kp            = PID_YAW_KP,
        .Ki            = PID_YAW_KI,
        .Kd            = PID_YAW_KD,
        .MaxOut        = MODE_STRAIGHT_MAX_CORRECTION,
        .IntegralLimit = 300.0f,
        .DeadBand      = 0.0f,
        .Improve       = PID_Integral_Limit
    };
    PIDInit(&s_yaw_pid, &cfg);

    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Straight_Exec(float base_pwm, float target_yaw, float current_yaw)
{
    /* 计算航向误差 (处理 ±180° 边界) */
    float yaw_error = target_yaw - current_yaw;
    if (yaw_error >  180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;

    /* PID 计算纠偏量 (dt=1.0 避免 D 项放大) */
    float turn_out = PIDCalculate(&s_yaw_pid, 0.0f, yaw_error, 1.0f);

    /* 纠偏限幅：与 base_pwm 成比例 */
    float max_cor = base_pwm * STRAIGHT_TURN_RATIO;
    if (turn_out >  max_cor) turn_out =  max_cor;
    if (turn_out < -max_cor) turn_out = -max_cor;

    /* 差速混合：由于物理极性改变，当前 yaw_error > 0 需要右轮加速拉回 */
    float pwm_left  = base_pwm - turn_out;
    float pwm_right = base_pwm + turn_out;

    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  pwm_left);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, pwm_right);
}
