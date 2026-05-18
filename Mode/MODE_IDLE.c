/**
 * @file  MODE_IDLE.c
 * @brief 空闲模式：电机输出 0，小车静止
 */
#include "MODE_IDLE.h"
#include "../APP/broker/broker.h"

void MODE_Idle_Init(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Idle_Exec(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}