/**
 * @file    message_center.h
 * @brief   Pub/Sub发布-订阅消息中心, 用于应用模块之间解耦通信
 * @origin  云台 modules/message_center/message_center.h
 * @note    100%可移植, 纯C链表+队列实现, 无硬件依赖
 *
 * 使用方法:
 *   Publisher_t  *pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));
 *   Subscriber_t *sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));
 *   PubPushMessage(pub, &cmd);       // 发布消息
 *   SubGetMessage(sub, &received);   // 获取消息
 */
#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stdint.h>

#define MAX_TOPIC_NAME_LEN 32   // 话题名最大长度
#define MAX_TOPIC_COUNT    12   // 最多支持的话题数量
#define QUEUE_SIZE          1   // 每个订阅者的消息队列深度

/* 订阅者 */
typedef struct mqt
{
    void    *queue[QUEUE_SIZE];
    uint8_t  data_len;
    uint8_t  front_idx;
    uint8_t  back_idx;
    uint8_t  temp_size;

    struct mqt *next_subs_queue;
} Subscriber_t;

/* 发布者 */
typedef struct ent
{
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;
    Subscriber_t *first_subs;
    struct ent *next_topic_node;
    uint8_t pub_registered_flag;
} Publisher_t;

/**
 * @brief 订阅话题
 * @param name     话题名称
 * @param data_len 消息数据长度, 建议用sizeof()
 * @return 订阅者实例指针
 */
Subscriber_t *SubRegister(char *name, uint8_t data_len);

/**
 * @brief 注册发布者
 * @param name     话题名称
 * @param data_len 消息数据长度
 * @return 发布者实例指针
 */
Publisher_t *PubRegister(char *name, uint8_t data_len);

/**
 * @brief 获取消息
 * @param sub      订阅者实例
 * @param data_ptr 接收数据的指针
 * @return 0=无新消息, 1=成功获取
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);

/**
 * @brief 发布消息给所有订阅者
 * @param pub      发布者实例
 * @param data_ptr 要发布的数据指针
 * @return 成功推送的订阅者数量
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);

#endif /* MESSAGE_CENTER_H */
