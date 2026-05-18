/**
 * @file  MODE_TURNING.c
 * @brief 原地转向模式：IMU 航向闭环转至目标角度
 *        差速驱动：左轮 = -turn, 右轮 = +turn 实现原地旋转
 */
#include "MODE_TURNING.h"
#include "../APP/broker/broker.h"
#include "../APP/algorithm/pid_controller.h"
#include "../APP/app_config.h"
#include <math.h>

static PIDInstance s_turn_pid;
static uint8_t     s_done = 0;

#define TURNING_STOP_DEADZONE  0.5f   /* 角度到位死区 (度) */

void MODE_Turning_Init(void)
{
    PID_Init_Config_s cfg = {
        .Kp            = PID_YAW_KP,
        .Ki            = 0.5f,           /* 转向需要较强积分消除稳态误差 */
        .Kd            = PID_TURNING_KD, /* 转向专用 D：强阻尼抑制过冲振荡 */
        .MaxOut        = TURNING_MAX_SPEED,
        .IntegralLimit = 300.0f,
        .DeadBand      = 0.0f,
        .Improve       = PID_Integral_Limit
    };
    PIDInit(&s_turn_pid, &cfg);
    s_done = 0;

    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Turning_Exec(float target_yaw, float current_yaw)
{
    /* 航向误差 (处理 180 边界) */
    float yaw_error = target_yaw - current_yaw;
    while (yaw_error >  180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    /* 到位判定 */
    if (fabsf(yaw_error) < TURNING_STOP_DEADZONE) {
        Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
        Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
        s_done = 1;
        return;
    }

    /* PID 计算 (target=误差, measure=0 -> 归零) */
    float turn_out = PIDCalculate(&s_turn_pid, 0.0f, yaw_error, 1.0f);

    /* 减速区：接近目标时按比例限速，防止高速冲过 */
    float abs_err = fabsf(yaw_error);
    if (abs_err < TURNING_DECEL_ZONE) {
        float scale = abs_err / TURNING_DECEL_ZONE;  /* 0~1 线性衰减 */
        float limit = TURNING_MAX_SPEED * scale;
        if (limit < TURNING_MIN_SPEED) limit = TURNING_MIN_SPEED;
        if (turn_out >  limit) turn_out =  limit;
        if (turn_out < -limit) turn_out = -limit;
    }

    /* 最小启动 PWM：克服静摩擦 */
    if (turn_out > 0 && turn_out < TURNING_MIN_SPEED)
        turn_out = TURNING_MIN_SPEED;
    else if (turn_out < 0 && turn_out > -TURNING_MIN_SPEED)
        turn_out = -TURNING_MIN_SPEED;

    /* 差速输出：由于物理极性改变，恢复原始逻辑 */
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  -turn_out);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT,  turn_out);
}

uint8_t MODE_Turning_IsDone(void)
{
    return s_done;
}
