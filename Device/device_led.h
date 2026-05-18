#ifndef DEVICE_LED_H
#define DEVICE_LED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief LED 底层控制函数指针
 */
typedef void (*Device_LED_WritePin_Func_t)(bool state);
typedef void (*Device_LED_TogglePin_Func_t)(void);

/**
 * @brief LED 模块实例结构体
 */
typedef struct {
    Device_LED_WritePin_Func_t  WritePin;
    Device_LED_TogglePin_Func_t TogglePin;
    bool                        IsOn;
} Device_LED_t;

/**
 * @brief LED 模块初始化配置结构体
 */
typedef struct {
    Device_LED_WritePin_Func_t  WritePin;
    Device_LED_TogglePin_Func_t TogglePin;
    bool                        InitState;
} Device_LED_Init_Config_t;

/* API 声明 */
void Device_LED_Init(Device_LED_t *led, Device_LED_Init_Config_t *config);
void Device_LED_Set(Device_LED_t *led, bool state);
void Device_LED_Toggle(Device_LED_t *led);

#endif /* DEVICE_LED_H */
