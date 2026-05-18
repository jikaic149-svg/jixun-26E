#include "device_key.h"

#define DEBOUNCE_TIME_S       0.05f   // 50ms 去抖（10 个 5ms tick，彻底稳定）
#define LONG_PRESS_TIME_S     0.80f   // 持续按下 0.8s 触发长按
#define DOUBLE_CLICK_GAP_S    0.25f   // 两次按下间隔 < 250ms 触发双击

static inline uint8_t is_active(const Device_Key_t *key)
{
    return key->ReadPin && (key->ReadPin() == key->ActiveLevel);
}

void Device_Key_Init(Device_Key_t *key, Device_Key_Init_Config_t *config)
{
    if (!key || !config) return;

    key->ReadPin     = config->ReadPin;
    key->ActiveLevel = config->ActiveLevel;
    key->State       = DEVICE_KEY_STATE_IDLE;
    key->Timer       = 0.0f;
    key->Event       = DEVICE_KEY_EVENT_NONE;
}

/* 简洁的事件型按键状态机：
 *
 *  IDLE --(low)--> DEBOUNCING --(20ms still low)--> PRESSED
 *  PRESSED --(release)--> WAIT_DOUBLE
 *  PRESSED --(0.8s)--> LONG_HELD (发 LONG_PRESS)
 *  LONG_HELD --(release)--> IDLE
 *  WAIT_DOUBLE --(low again)--> 发 DOUBLE_CLICK, 回 PRESSED
 *  WAIT_DOUBLE --(>300ms)--> 发 SHORT_PRESS, 回 IDLE
 */
void Device_Key_Update(Device_Key_t *key, float dt_s)
{
    if (!key || !key->ReadPin) return;

    uint8_t active = is_active(key);
    key->Timer += dt_s;

    switch (key->State) {

    case DEVICE_KEY_STATE_IDLE:
        if (active) {
            key->State = DEVICE_KEY_STATE_DEBOUNCING;
            key->Timer = 0.0f;
        }
        break;

    case DEVICE_KEY_STATE_DEBOUNCING:
        if (!active) {                       // 抖动，回 IDLE
            key->State = DEVICE_KEY_STATE_IDLE;
            key->Timer = 0.0f;
        } else if (key->Timer >= DEBOUNCE_TIME_S) {
            key->State = DEVICE_KEY_STATE_PRESSED;
            /* Timer 继续累加，用于检测长按 */
        }
        break;

    case DEVICE_KEY_STATE_PRESSED:
        if (!active) {                       // 短促松开：等待是否构成双击
            key->State = DEVICE_KEY_STATE_WAIT_DOUBLE;
            key->Timer = 0.0f;
        } else if (key->Timer >= LONG_PRESS_TIME_S) {
            key->Event = DEVICE_KEY_EVENT_LONG_PRESS;
            key->State = DEVICE_KEY_STATE_LONG_HELD;
        }
        break;

    case DEVICE_KEY_STATE_LONG_HELD:
        if (!active) {
            key->State = DEVICE_KEY_STATE_IDLE;
            key->Timer = 0.0f;
        }
        break;

    case DEVICE_KEY_STATE_WAIT_DOUBLE:
        if (active) {                        // 第二次按下 -> 双击
            key->Event = DEVICE_KEY_EVENT_DOUBLE_CLICK;
            key->State = DEVICE_KEY_STATE_LONG_HELD; // 等松手，不再产生事件
            key->Timer = 0.0f;
        } else if (key->Timer >= DOUBLE_CLICK_GAP_S) {
            key->Event = DEVICE_KEY_EVENT_SHORT_PRESS;
            key->State = DEVICE_KEY_STATE_IDLE;
            key->Timer = 0.0f;
        }
        break;
    }
}

Device_Key_Event_e Device_Key_GetEvent(Device_Key_t *key)
{
    return key ? key->Event : DEVICE_KEY_EVENT_NONE;
}

void Device_Key_ClearEvent(Device_Key_t *key)
{
    if (key) key->Event = DEVICE_KEY_EVENT_NONE;
}
