/**
 * @file broker.c
 * @brief Application层：轻量级消息总线机制实现
 *        (Rule 4: 零耦合消息总线, Rule 5/6: 临界区保护线程安全)
 */
#include "broker.h"

/* Cortex-M 临界区宏 (纯内联汇编，兼容 tiarmclang / GCC / ARMCC) */
#define BROKER_ENTER_CRITICAL()  uint32_t _primask; \
    __asm volatile ("MRS %0, PRIMASK\n CPSID i" : "=r"(_primask))
#define BROKER_EXIT_CRITICAL()   \
    __asm volatile ("MSR PRIMASK, %0" :: "r"(_primask))

/* 内存分配池 (因为话题总数明确且单片读写，不需要 malloc/动态映射) */
static volatile float   _float_topics[TOPIC_MAX_NUM];
static volatile int32_t _int_topics[TOPIC_MAX_NUM];

void Broker_Init(void)
{
    for (int i = 0; i < TOPIC_MAX_NUM; i++) {
        _float_topics[i] = 0.0f;
        _int_topics[i] = 0;
    }
    // 特殊初始化：颜色和形状默认为-1 (未知)
    _int_topics[TOPIC_VISION_COLOR] = -1;
    _int_topics[TOPIC_VISION_SHAPE] = -1;
}

void Broker_Publish_Float(TopicID_e topic, float value)
{
    if (topic < TOPIC_MAX_NUM) {
        BROKER_ENTER_CRITICAL();
        _float_topics[topic] = value;
        BROKER_EXIT_CRITICAL();
    }
}

float Broker_Subscribe_Float(TopicID_e topic)
{
    float val = 0.0f;
    if (topic < TOPIC_MAX_NUM) {
        BROKER_ENTER_CRITICAL();
        val = _float_topics[topic];
        BROKER_EXIT_CRITICAL();
    }
    return val;
}

void Broker_Publish_Int(TopicID_e topic, int32_t value)
{
    if (topic < TOPIC_MAX_NUM) {
        BROKER_ENTER_CRITICAL();
        _int_topics[topic] = value;
        BROKER_EXIT_CRITICAL();
    }
}

int32_t Broker_Subscribe_Int(TopicID_e topic)
{
    int32_t val = 0;
    if (topic < TOPIC_MAX_NUM) {
        BROKER_ENTER_CRITICAL();
        val = _int_topics[topic];
        BROKER_EXIT_CRITICAL();
    }
    return val;
}
