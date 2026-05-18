/**
 * @file  MODE_BRAKE.c
 * @brief 制动模式：输出 0 目标速度，由速度闭环 PID 主动制动
 *        利用编码器实测轮速判定是否停稳，替代旧版纯时间超时
 */
#include "MODE_BRAKE.h"
#include "../APP/broker/broker.h"
#include "../APP/app_config.h"
#include <math.h>

static uint8_t  s_stable_cnt = 0;   /* 连续低速计数 */
static uint16_t s_brake_tick = 0;   /* 安全超时计数 */

#define BRAKE_TIMEOUT_TICKS  100    /* 安全上限 500ms，防止卡死 */

void MODE_Brake_Init(void)
{
    s_stable_cnt = 0;
    s_brake_tick = 0;
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Brake_Exec(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
    s_brake_tick++;

    /* 读取编码器实测轮速，判断是否真正停稳 */
    float spd_l = Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
    float spd_r = Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);
    if (fabsf(spd_l) < BRAKE_SPEED_THRESHOLD && fabsf(spd_r) < BRAKE_SPEED_THRESHOLD) {
        s_stable_cnt++;
    } else {
        s_stable_cnt = 0;
    }
}

uint8_t MODE_Brake_IsDone(void)
{
    return (s_stable_cnt >= BRAKE_STABLE_COUNT || s_brake_tick >= BRAKE_TIMEOUT_TICKS) ? 1 : 0;
}
