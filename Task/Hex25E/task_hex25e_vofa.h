#ifndef TASK_HEX25E_VOFA_H
#define TASK_HEX25E_VOFA_H

#include "hex25e_config.h"

#include "device_serial.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t tx_count;
    uint32_t tx_drop_count;
} Task_Hex25E_VofaStats;

typedef struct {
    Device_Serial *serial;
    uint8_t tx_buffer[HEX25E_CONFIG_VOFA_TX_BUFFER_SIZE];
    Task_Hex25E_VofaStats stats;
    bool initialized;
} Task_Hex25E_Vofa;

bool Task_Hex25E_Vofa_Init(
    Task_Hex25E_Vofa *vofa, Device_Serial *serial);
bool Task_Hex25E_Vofa_SendCsvFloat(
    Task_Hex25E_Vofa *vofa,
    const float *channels,
    size_t channel_count);
bool Task_Hex25E_Vofa_SendText(
    Task_Hex25E_Vofa *vofa, const char *text);
bool Task_Hex25E_Vofa_GetStats(
    const Task_Hex25E_Vofa *vofa,
    Task_Hex25E_VofaStats *stats);

#endif
