#include "app_ui.h"
#include "app_config.h"
#include "../Device/device_init.h"
#include "../Device/device_ws2812.h"
#include "broker/broker.h"
#include "../Mode/mode_manager.h"

static uint8_t ws2812_state = 0;

void App_UI_Init(void) {
    // 初始状态：上电RGB不亮灯 (测试需要)
    Device_WS2812_Clear();
    Device_WS2812_Show();
    
    Broker_Publish_Int(TOPIC_MOTOR_ENABLE, 0); // 上电默认不使能电机
}

void App_UI_Update(float dt_s) {
    // 从总线获取并处理按键事件
    Device_Key_Event_e event = (Device_Key_Event_e)Broker_Subscribe_Int(TOPIC_KEY_EVENT);
    if (event != DEVICE_KEY_EVENT_NONE) {
        // 清除总线上的事件，避免重复处理
        Broker_Publish_Int(TOPIC_KEY_EVENT, (int32_t)DEVICE_KEY_EVENT_NONE);

        switch (event) {
            case DEVICE_KEY_EVENT_SHORT_PRESS:
                // 单击：进入 test 任务，亮红灯，使能电机
                Device_JY61P_TareYaw(); // 软件清零陀螺仪航向角
                Mode_SetTask(1); // 切入测试流水线
                Broker_Publish_Int(TOPIC_MOTOR_ENABLE, 1); // 使能电机输出电流
                
                Device_WS2812_SetAll(255, 0, 0); // 红光
                Device_WS2812_Show();
                break;

            case DEVICE_KEY_EVENT_DOUBLE_CLICK:
                // 双击：停止任务，不再输出电流控制电机，亮蓝灯
                Mode_SetIdle(); // 切回 IDLE 模式
                Broker_Publish_Int(TOPIC_MOTOR_ENABLE, 0); // 禁用电机物理输出
                
                Device_WS2812_SetAll(0, 0, 255); // 蓝光
                Device_WS2812_Show();
                break;

            case DEVICE_KEY_EVENT_LONG_PRESS:
                // 长按：可以用来关灯或做其他事
                Device_WS2812_Clear();
                Device_WS2812_Show();
                break;

            default:
                break;
        }
    }
}
