#ifndef DEVICE_KEY_INPUT_H
#define DEVICE_KEY_INPUT_H

#include "device_key.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    Device_Key_Config key;
    size_t input_index;
} Device_KeyInput_Config;

typedef struct {
    Device_Key key;
    const void *backend;
    bool initialized;
} Device_KeyInput;

bool Device_KeyInput_Init(
    Device_KeyInput *input, const Device_KeyInput_Config *config,
    uint32_t now_ms);
bool Device_KeyInput_UpdatePressedEdge(
    Device_KeyInput *input, uint32_t now_ms, bool *pressed_edge);

#endif
