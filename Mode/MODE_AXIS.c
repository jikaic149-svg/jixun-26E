/**
 * @file  MODE_AXIS.c
 * @brief 坐标导航模块占位实现
 * @note  当前项目无编码器反馈, 无法进行里程计定位
 *        此模块所有功能均返回"已完成"以避免状态机卡死
 *        后续若加装编码器, 可在此基础上实现完整功能
 */
#include "MODE_AXIS.h"
#include "../APP/broker/broker.h"

void MODE_Axis_Init(float target_x_mm, float target_y_mm)
{
    (void)target_x_mm;
    (void)target_y_mm;
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

void MODE_Axis_Exec(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
}

uint8_t MODE_Axis_IsDone(void)
{
    return 1;
}
