/**
 * @file  MODE_TRACKING.c
 * @brief 恒速循迹：8 路红外 PID 横向纠偏，基速恒定（不做自适应变速）
 *        丢线时保持上一拍差速输出，由上层 watchdog 决定何时介入
 *        输出语义：mm/s（下游速度环吃 mm/s）
 */
#include "MODE_TRACKING.h"
#include "Application/broker/broker.h"
#include "config.h"
#include <math.h>

/* ======================== 内部状态 ======================== */
static float s_last_error = 0.0f;
static float s_last_turn  = 0.0f;
static uint8_t s_was_lost = 1;

void MODE_Tracking_Init(void)
{
    s_last_error = 0.0f;
    s_last_turn  = 0.0f;
    s_was_lost   = 1;

    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
}

/**
 * @param base_speed     基础前进速度 (mm/s)，恒定
 * @param current_error  循迹误差，999 表示丢线
 */
void MODE_Tracking_Exec(float base_speed, float current_error)
{
    float turn_out;

    if (current_error != 999.0f) {
        /* 丢线 → 寻线时清微分跳变 */
        if (s_was_lost) {
            s_last_error = current_error;
            s_was_lost   = 0;
        }

        float derivative = current_error - s_last_error;
        turn_out = PID_TRACK_KP * current_error + PID_TRACK_KD * derivative;
        s_last_error = current_error;
        
        /* 引入 WZ (Yaw角速度) 阻尼，抑制直线循迹左右横跳晃动 */
        float wz = Broker_Subscribe_Float(TOPIC_JY61P_WZ);
        turn_out -= wz * PID_TRACK_WZ_KD;

        /* 转向限幅（占基速比例） */
        float max_turn = base_speed * TRACK_MAX_TURN_RATIO;
        if (turn_out >  max_turn) turn_out =  max_turn;
        if (turn_out < -max_turn) turn_out = -max_turn;

        s_last_turn = turn_out;
    } else {
        /* 丢线：保持上一拍差速（短时惯性穿越），基速不变 */
        s_was_lost = 1;
        turn_out   = s_last_turn;
    }

    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  base_speed - turn_out);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, base_speed + turn_out);
}
