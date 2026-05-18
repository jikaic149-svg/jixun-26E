/**
 * @file  MODE_DONE.c
 * @brief 完成态：任务结束，电机永久停机
 */
#include "MODE_DONE.h"
#include "../APP/broker/broker.h"

void MODE_Done_Init(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Done_Exec(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}
