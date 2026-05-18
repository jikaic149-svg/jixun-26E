/**
 * @file jy61p.h
 * @brief JY61P传感器硬件驱动 (纯解析脱离业务)
 */
#ifndef DEVICE_JY61P_H
#define DEVICE_JY61P_H

#include <stdint.h>
#include <stdbool.h>
#include "../APP/framework/daemon.h"

#define JY61P_RX_BUFFER_SIZE 256

/* 通信接收 FIFO 环形缓冲区 */
typedef struct {
    uint8_t buffer[JY61P_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} JY61P_RingBuffer_t;

/* 解析状态枚举 */
typedef enum {
    JY61P_WAIT_HEADER1 = 0,
    JY61P_WAIT_HEADER2,
    JY61P_RECEIVE_DATA
} JY61P_ParseState_e;

/* JY61P 面向对象实例化结构 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float yaw_offset;  // 软件清零偏移量
    float ax;          // X轴加速度 m/s^2
    float ay;          // Y轴加速度 m/s^2
    float az;          // Z轴加速度 m/s^2
    float wx;          // X轴角速度 °/s
    float wy;          // Y轴角速度 °/s
    float wz;          // Z轴角速度 °/s（串级 PID 内环反馈）

    bool data_ready;   // 至少成功解析过一帧角度包 (0x53)

    JY61P_ParseState_e state;
    uint8_t rx_frame[11];
    uint8_t frame_index;
    uint8_t current_type;
    JY61P_RingBuffer_t fifo;
    DaemonInstance *daemon;
} JY61P_Instance;

void JY61P_Init(JY61P_Instance *inst);
void JY61P_FIFO_Write(JY61P_Instance *inst, uint8_t data);
void JY61P_Parse_Task(JY61P_Instance *inst);
void JY61P_TareYaw(JY61P_Instance *inst);

#endif /* DEVICE_JY61P_H */
