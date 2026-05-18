#ifndef DEVICE_KEY_H
#define DEVICE_KEY_H

#include <stdint.h>

/**
 * @brief 按键事件枚举（事件队列中的输出）
 */
typedef enum {
    DEVICE_KEY_EVENT_NONE = 0,
    DEVICE_KEY_EVENT_SHORT_PRESS,
    DEVICE_KEY_EVENT_LONG_PRESS,
    DEVICE_KEY_EVENT_DOUBLE_CLICK,
} Device_Key_Event_e;

/**
 * @brief 按键内部状态机
 */
typedef enum {
    DEVICE_KEY_STATE_IDLE = 0,         // 等待按下
    DEVICE_KEY_STATE_DEBOUNCING,       // 检测到电平变化，去抖
    DEVICE_KEY_STATE_PRESSED,          // 已确认按下，等待松开或长按超时
    DEVICE_KEY_STATE_LONG_HELD,        // 已触发长按，等松开
    DEVICE_KEY_STATE_WAIT_DOUBLE,      // 松开后等下一次短按以构成双击
} Device_Key_State_e;

typedef uint8_t (*Device_Key_ReadPin_Func_t)(void);

/**
 * @brief 按键模块实例
 */
typedef struct {
    /* 配置 */
    Device_Key_ReadPin_Func_t  ReadPin;
    uint8_t                    ActiveLevel;

    /* 状态机 */
    Device_Key_State_e         State;
    float                      Timer;        // 当前状态持续时间（秒）

    /* 事件输出 */
    Device_Key_Event_e         Event;
} Device_Key_t;

typedef struct {
    Device_Key_ReadPin_Func_t  ReadPin;
    uint8_t                    ActiveLevel;
} Device_Key_Init_Config_t;

void               Device_Key_Init(Device_Key_t *key, Device_Key_Init_Config_t *config);
void               Device_Key_Update(Device_Key_t *key, float dt_s);
Device_Key_Event_e Device_Key_GetEvent(Device_Key_t *key);
void               Device_Key_ClearEvent(Device_Key_t *key);

#endif /* DEVICE_KEY_H */
