/**
 * @file broker.h
 * @brief Application层：基于多话题分发的轻量级消息总线机制 (Pub/Sub)
 */
#ifndef APP_BROKER_H
#define APP_BROKER_H

#include <stdint.h>

/* 定义系统支持的所有话题 */
typedef enum {
    TOPIC_TARGET_SPEED_LEFT = 0,
    TOPIC_TARGET_SPEED_RIGHT,

    /* 惯性导航相关话题 */
    TOPIC_JY61P_ROLL,
    TOPIC_JY61P_PITCH,
    TOPIC_JY61P_YAW,
    TOPIC_JY61P_AX,
    TOPIC_JY61P_AY,
    TOPIC_JY61P_AZ,
    TOPIC_JY61P_WX,
    TOPIC_JY61P_WY,
    TOPIC_JY61P_WZ,
    TOPIC_INS_SPEED_X,
    TOPIC_INS_MILEAGE_X,

    /* Mode 状态机相关话题 */
    TOPIC_CAR_MODE,         /* 当前运行模式 (CarMode_e) */
    TOPIC_TRACK_ERROR,      /* 循迹传感器误差 (float) */

    /* Task 层话题 */
    TOPIC_TASK_ID,          /* 当前执行的任务编号 (0=无, 1~4) */

    /* 编码器实测速度 (mm/s) */
    TOPIC_SPEED_LEFT,       /* 左轮实测线速度 */
    TOPIC_SPEED_RIGHT,      /* 右轮实测线速度 */

    /* 速度闭环目标轮速 (mm/s) */
    TOPIC_TARGET_WHEEL_SPEED_LEFT,
    TOPIC_TARGET_WHEEL_SPEED_RIGHT,

    /* 编码器累积里程 (mm) */
    TOPIC_DISTANCE_LEFT,
    TOPIC_DISTANCE_RIGHT,

    /* 底层执行器控制话题 */
    TOPIC_PWM_LEFT,         /* 左电机输出PWM (-100~100) */
    TOPIC_PWM_RIGHT,        /* 右电机输出PWM (-100~100) */

    /* UI 及交互话题 */
    TOPIC_KEY_EVENT,        /* 用户按键事件 (对应 Device_Key_Event_e) */

    TOPIC_MOTOR_ENABLE,     /* 电机使能 (0=Sleep, 1=Enable) */

    /* 视觉传感器话题 */
    TOPIC_VISION_TURN,      /* 视觉建议转向 (0=左转, 1=右转) */
    TOPIC_VISION_COLOR,     /* 视觉目标颜色 (0=绿, 1=红) */
    TOPIC_VISION_SHAPE,     /* 视觉目标形状 (0=方, 1=圆) */

    TOPIC_MAX_NUM
} TopicID_e;

/* 初始化总线内存数据空间 */
void Broker_Init(void);

/* 基础浮点型深拷贝收发 (32bit 无锁访问) */
void  Broker_Publish_Float(TopicID_e topic, float value);
float Broker_Subscribe_Float(TopicID_e topic);

/* 基础整型深拷贝收发 (32bit 无锁访问) */
void    Broker_Publish_Int(TopicID_e topic, int32_t value);
int32_t Broker_Subscribe_Int(TopicID_e topic);

#endif /* APP_BROKER_H */
