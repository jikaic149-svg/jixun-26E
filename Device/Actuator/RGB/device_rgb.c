#include "device_rgb.h"

#include "bsp_rgb.h"

#include <stddef.h>

static bool same_color(Device_RGB_Color left, Device_RGB_Color right)
{
    return (left.red == right.red) &&
        (left.green == right.green) &&
        (left.blue == right.blue);
}

bool Device_RGB_DefaultConfig(size_t index, Device_RGB_Config *config)
{
    if ((config == NULL) || (index >= BSP_RGB_Count())) {
        return false;
    }
    *config = (Device_RGB_Config){
        .pixel_index = index,
    };
    return true;
}

bool Device_RGB_Init(Device_RGB *rgb, const Device_RGB_Config *config)
{
    const BSP_RGB_Pixel off = {0U, 0U, 0U};

    if (rgb != NULL) {
        *rgb = (Device_RGB){0};
    }
    if ((rgb == NULL) || (config == NULL) ||
        (config->pixel_index >= BSP_RGB_Count()) ||
        !BSP_RGB_SetPixel(config->pixel_index, off) ||
        !BSP_RGB_Show()) {
        return false;
    }
    rgb->config = *config;
    rgb->initialized = true;
    return true;
}

Device_RGB_Result Device_RGB_SetColor(
    Device_RGB *rgb, Device_RGB_Color color)
{
    const BSP_RGB_Pixel pixel = {
        .red = color.red,
        .green = color.green,
        .blue = color.blue,
    };

    if ((rgb == NULL) || !rgb->initialized) {
        return DEVICE_RGB_RESULT_INVALID_ARGUMENT;
    }
    if (same_color(rgb->color, color)) {
        return DEVICE_RGB_RESULT_OK;
    }
    if (!BSP_RGB_SetPixel(rgb->config.pixel_index, pixel) ||
        !BSP_RGB_Show()) {
        return DEVICE_RGB_RESULT_BACKEND_ERROR;
    }
    rgb->color = color;
    return DEVICE_RGB_RESULT_OK;
}

Device_RGB_Result Device_RGB_Off(Device_RGB *rgb)
{
    return Device_RGB_SetColor(rgb, (Device_RGB_Color){0U, 0U, 0U});
}

bool Device_RGB_GetColor(
    const Device_RGB *rgb, Device_RGB_Color *color)
{
    if ((rgb == NULL) || (color == NULL) || !rgb->initialized) {
        return false;
    }
    *color = rgb->color;
    return true;
}
