#include "device_key_input.h"

#include "bsp_key.h"

bool Device_KeyInput_Init(
    Device_KeyInput *input, const Device_KeyInput_Config *config,
    uint32_t now_ms)
{
    BSP_Key_Handle handle;
    bool pressed;

    if (input != NULL) {
        *input = (Device_KeyInput){0};
    }
    if ((input == NULL) || (config == NULL)) {
        return false;
    }
    handle = BSP_Key_GetHandle(config->input_index);
    if ((handle == NULL) || !BSP_Key_Read(handle, &pressed) ||
        !Device_Key_Init(&input->key, &config->key, pressed, now_ms)) {
        return false;
    }
    input->backend = handle;
    input->initialized = true;
    return true;
}

bool Device_KeyInput_UpdatePressedEdge(
    Device_KeyInput *input, uint32_t now_ms, bool *pressed_edge)
{
    bool raw_pressed;
    bool was_pressed;
    bool is_pressed;

    if (pressed_edge != NULL) {
        *pressed_edge = false;
    }
    if ((input == NULL) || !input->initialized ||
        (pressed_edge == NULL) ||
        !BSP_Key_Read((BSP_Key_Handle)input->backend, &raw_pressed)) {
        return false;
    }
    was_pressed = Device_Key_IsPressed(&input->key);
    if (!Device_Key_Update(&input->key, raw_pressed, now_ms)) {
        return false;
    }
    is_pressed = Device_Key_IsPressed(&input->key);
    *pressed_edge = !was_pressed && is_pressed;
    Device_Key_ClearEvents(&input->key);
    return true;
}
