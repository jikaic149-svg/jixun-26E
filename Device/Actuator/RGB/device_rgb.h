#ifndef DEVICE_RGB_H
#define DEVICE_RGB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEVICE_RGB_RESULT_OK = 0,
    DEVICE_RGB_RESULT_INVALID_ARGUMENT,
    DEVICE_RGB_RESULT_BACKEND_ERROR,
} Device_RGB_Result;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Device_RGB_Color;

typedef struct {
    size_t pixel_index;
} Device_RGB_Config;

typedef struct {
    Device_RGB_Config config;
    Device_RGB_Color color;
    bool initialized;
} Device_RGB;

bool Device_RGB_DefaultConfig(size_t index, Device_RGB_Config *config);
bool Device_RGB_Init(Device_RGB *rgb, const Device_RGB_Config *config);
Device_RGB_Result Device_RGB_SetColor(
    Device_RGB *rgb, Device_RGB_Color color);
Device_RGB_Result Device_RGB_Off(Device_RGB *rgb);
bool Device_RGB_GetColor(
    const Device_RGB *rgb, Device_RGB_Color *color);

#endif
