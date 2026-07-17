#include "device_led.h"

#include "bsp_led.h"

#include <stddef.h>

bool Device_LED_DefaultConfig(size_t index, Device_LED_Config *config)
{
    if ((config == NULL) || (index >= BSP_LED_Count())) {
        return false;
    }
    *config = (Device_LED_Config){
        .backend_index = index,
    };
    return true;
}

bool Device_LED_Init(Device_LED *led, const Device_LED_Config *config)
{
    if (led != NULL) {
        *led = (Device_LED){0};
    }
    if ((led == NULL) || (config == NULL) ||
        (config->backend_index >= BSP_LED_Count()) ||
        !BSP_LED_Set(config->backend_index, false)) {
        return false;
    }
    led->config = *config;
    led->initialized = true;
    return true;
}

Device_LED_Result Device_LED_Set(Device_LED *led, bool on)
{
    if ((led == NULL) || !led->initialized) {
        return DEVICE_LED_RESULT_INVALID_ARGUMENT;
    }
    if (led->on == on) {
        return DEVICE_LED_RESULT_OK;
    }
    if (!BSP_LED_Set(led->config.backend_index, on)) {
        return DEVICE_LED_RESULT_BACKEND_ERROR;
    }
    led->on = on;
    return DEVICE_LED_RESULT_OK;
}

bool Device_LED_IsOn(const Device_LED *led)
{
    return (led != NULL) && led->initialized && led->on;
}
