#ifndef DEVICE_LED_H
#define DEVICE_LED_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    DEVICE_LED_RESULT_OK = 0,
    DEVICE_LED_RESULT_INVALID_ARGUMENT,
    DEVICE_LED_RESULT_BACKEND_ERROR,
} Device_LED_Result;

typedef struct {
    size_t backend_index;
} Device_LED_Config;

typedef struct {
    Device_LED_Config config;
    bool on;
    bool initialized;
} Device_LED;

bool Device_LED_DefaultConfig(size_t index, Device_LED_Config *config);
bool Device_LED_Init(Device_LED *led, const Device_LED_Config *config);
Device_LED_Result Device_LED_Set(Device_LED *led, bool on);
bool Device_LED_IsOn(const Device_LED *led);

#endif
