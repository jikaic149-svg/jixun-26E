#include "task_hex25e_vofa.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void increment_saturating(uint32_t *value)
{
    if ((value != NULL) && (*value != UINT32_MAX)) {
        ++(*value);
    }
}

static bool append_char(
    uint8_t *buffer, size_t capacity, size_t *used, char value)
{
    if ((buffer == NULL) || (used == NULL) || (*used >= capacity)) {
        return false;
    }
    buffer[(*used)++] = (uint8_t)value;
    return true;
}

static bool append_u32(
    uint8_t *buffer, size_t capacity, size_t *used, uint32_t value)
{
    char reverse[10];
    size_t count = 0U;

    do {
        reverse[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(reverse)));
    while (count > 0U) {
        if (!append_char(buffer, capacity, used, reverse[--count])) {
            return false;
        }
    }
    return true;
}

static bool append_fixed_3(
    uint8_t *buffer, size_t capacity, size_t *used, float value)
{
    float magnitude;
    uint32_t scaled;
    uint32_t fraction;

    /* FireWater requires a numeric field in every fixed CSV column.  Preserve
     * validity through the adjacent online/fault columns and never inject a
     * non-numeric token into this product telemetry row. */
    if (!isfinite(value)) {
        value = 0.0F;
    }
    if (value < 0.0F) {
        if (!append_char(buffer, capacity, used, '-')) {
            return false;
        }
        magnitude = -value;
    }
    else {
        magnitude = value;
    }
    if (magnitude > 4294967.0F) {
        magnitude = 4294967.0F;
    }
    scaled = (uint32_t)(magnitude * 1000.0F + 0.5F);
    fraction = scaled % 1000U;
    return append_u32(buffer, capacity, used, scaled / 1000U) &&
        append_char(buffer, capacity, used, '.') &&
        append_char(buffer, capacity, used,
            (char)('0' + (fraction / 100U))) &&
        append_char(buffer, capacity, used,
            (char)('0' + ((fraction / 10U) % 10U))) &&
        append_char(buffer, capacity, used,
            (char)('0' + (fraction % 10U)));
}

bool Task_Hex25E_Vofa_Init(
    Task_Hex25E_Vofa *vofa, Device_Serial *serial)
{
    if (vofa != NULL) {
        *vofa = (Task_Hex25E_Vofa){0};
    }
    if ((vofa == NULL) || (serial == NULL) || !serial->initialized) {
        return false;
    }
    vofa->serial = serial;
    vofa->initialized = true;
    return true;
}

bool Task_Hex25E_Vofa_SendCsvFloat(
    Task_Hex25E_Vofa *vofa,
    const float *channels,
    size_t channel_count)
{
    size_t used = 0U;

    if ((vofa == NULL) || !vofa->initialized ||
        (vofa->serial == NULL) || (channels == NULL) ||
        (channel_count == 0U) ||
        (channel_count > HEX25E_CONFIG_VOFA_MAX_CHANNELS)) {
        return false;
    }
    for (size_t index = 0U; index < channel_count; ++index) {
        if (((index > 0U) && !append_char(vofa->tx_buffer,
                sizeof(vofa->tx_buffer), &used, ',')) ||
            !append_fixed_3(vofa->tx_buffer,
                sizeof(vofa->tx_buffer), &used, channels[index])) {
            increment_saturating(&vofa->stats.tx_drop_count);
            return false;
        }
    }
    if (!append_char(vofa->tx_buffer,
            sizeof(vofa->tx_buffer), &used, '\r') ||
        !append_char(vofa->tx_buffer,
            sizeof(vofa->tx_buffer), &used, '\n') ||
        (Device_Serial_WriteBuffered(vofa->serial,
            vofa->tx_buffer, used) != DEVICE_SERIAL_RESULT_OK)) {
        increment_saturating(&vofa->stats.tx_drop_count);
        return false;
    }
    increment_saturating(&vofa->stats.tx_count);
    return true;
}

bool Task_Hex25E_Vofa_SendText(
    Task_Hex25E_Vofa *vofa, const char *text)
{
    size_t length;

    if ((vofa == NULL) || !vofa->initialized ||
        (vofa->serial == NULL) || (text == NULL)) {
        return false;
    }
    length = strlen(text);
    if ((length == 0U) ||
        (Device_Serial_WriteBuffered(vofa->serial,
            (const uint8_t *)text, length) != DEVICE_SERIAL_RESULT_OK)) {
        increment_saturating(&vofa->stats.tx_drop_count);
        return false;
    }
    increment_saturating(&vofa->stats.tx_count);
    return true;
}

bool Task_Hex25E_Vofa_GetStats(
    const Task_Hex25E_Vofa *vofa,
    Task_Hex25E_VofaStats *stats)
{
    if ((vofa == NULL) || !vofa->initialized || (stats == NULL)) {
        return false;
    }
    *stats = vofa->stats;
    return true;
}
