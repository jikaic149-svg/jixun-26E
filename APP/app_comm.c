#include "app_comm.h"
#include "app_config.h"
#include "app_motor.h"
#include "../BSP/bsp_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../Device/device_init.h"
#include "../Device/device_ws2812.h"
#include "broker/broker.h"

static uint32_t comm_timer = 0;
static char tx_buf[256];

/* 收到 6 字节二进制视觉命令包回调 (AA 55 Turn Color Shape Checksum) */
static void Comm_RxCmdCallback(uint8_t turn, uint8_t color, uint8_t shape) {
    Broker_Publish_Int(TOPIC_VISION_TURN, turn);
    Broker_Publish_Int(TOPIC_VISION_COLOR, color);
    Broker_Publish_Int(TOPIC_VISION_SHAPE, shape);
    
    // 收到视觉特征时，不再在此处直接点亮灯光！
    // 否则视野里闪过任何干扰色块都会让灯乱闪。
    // 灯光的点亮将严格推迟到小车真正到达路口并决定转向的那一刻（由 GetNextTurnAngle 负责）！
}

/* 收到文本行命令回调 (在中断上下文中被调用，必须极度轻量化) */
static void Comm_RxLineCallback(const char *line, uint16_t len) {
    // 使用静态变量记住上一次的目标速度
    static float left_v = 0;
    static float right_v = 0;
    bool updated = false;

    // 处理单独的 "PWM" 指令开启自动辨识序列
    if (strncmp(line, "PWM", 3) == 0 && (len == 3 || line[3] == '\r' || line[3] == '\n')) {
        char ver_msg[] = "\r\n=== FIRMWARE V2: CROSS-COUPLED FIX ===\r\n";
        BSP_UART1_SendDMA((uint8_t*)ver_msg, sizeof(ver_msg)-1);
        App_Motor_StartIdentSequence();
        return;
    }

    // ==== 测试通信专用逻辑：收到1亮红灯，收到2亮绿灯 ====
    if (line[0] == '1') {
        Device_WS2812_SetAll(255, 0, 0); // 收到1，亮红灯
        Device_WS2812_Show();
        return;
    } else if (line[0] == '2') {
        Device_WS2812_SetAll(0, 255, 0); // 收到2，亮绿灯
        Device_WS2812_Show();
        return;
    }
    // =================================================
    
    // 处理急停指令
    if (strncmp(line, "STOP", 4) == 0 || strncmp(line, "stop", 4) == 0) {
        App_Motor_Stop();
        left_v = 0;
        right_v = 0;
        return;
    }

    // 支持格式 "L500R-500" 或 "R100 L200"
    const char *pL = strchr(line, 'L');
    if (pL != NULL) {
        left_v = strtof(pL + 1, NULL);
        updated = true;
    }
    
    const char *pR = strchr(line, 'R');
    if (pR != NULL) {
        right_v = strtof(pR + 1, NULL);
        updated = true;
    }

    if (updated) {
        App_Motor_SetTargetSpeed(left_v, right_v);
    }
}

void App_Comm_Init(void) {
    comm_timer = 0;
    Bluetooth_RegisterLineCallback(&bt_inst, Comm_RxLineCallback);
    Bluetooth_RegisterRxCallback(&bt_inst, Comm_RxCmdCallback);
}

void App_Comm_Update(uint32_t elapsed_ms) {
    comm_timer += elapsed_ms;
    
    // 维护一个局部系统运行时间计数器用于打印
    static uint32_t sys_time_ms = 0;
    sys_time_ms += elapsed_ms;

    if (comm_timer >= COMM_REPORT_INTERVAL_MS) {
        comm_timer = 0;
        
        float tar_L = Broker_Subscribe_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT);
        float tar_R = Broker_Subscribe_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT);
        float cur_L = Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
        float cur_R = Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);
        float dist_L = Broker_Subscribe_Float(TOPIC_DISTANCE_LEFT);
        float dist_R = Broker_Subscribe_Float(TOPIC_DISTANCE_RIGHT);
        float yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
        
        // 输出格式：时间戳，目标轮速，实际轮速，总里程计，Yaw
        float odo = (dist_L + dist_R) / 2.0f;
        int len = snprintf(tx_buf, sizeof(tx_buf), 
            "T:%u,TarL:%.1f,TarR:%.1f,CurL:%.1f,CurR:%.1f,Odo:%.1f,Yaw:%.2f\r\n", 
            sys_time_ms, tar_L, tar_R, cur_L, cur_R, odo, yaw);
            
        if (len > 0) {
            if (len >= sizeof(tx_buf)) len = sizeof(tx_buf) - 1; // 严防越界
            BSP_UART1_SendString((char*)tx_buf); // 临时改为阻塞发送，用于排查 DMA 问题
        }
    }
}
