#ifndef BSP_LINE_SENSOR_H
#define BSP_LINE_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_LINE_SENSOR_MAX_CHANNELS 16U
#define BSP_LINE_SENSOR_VALUE_MIN     0U
#define BSP_LINE_SENSOR_VALUE_MAX     10000U

typedef const struct BSP_LineSensor_Resource *BSP_LineSensor_Handle;

typedef enum {
    BSP_LINE_SENSOR_CHANNEL_DIGITAL = 0,
    BSP_LINE_SENSOR_CHANNEL_ANALOG,
} BSP_LineSensor_ChannelKind;

typedef struct {
    size_t channel_count;
    BSP_LineSensor_ChannelKind channel_kind[BSP_LINE_SENSOR_MAX_CHANNELS];
} BSP_LineSensor_Info;

typedef struct {
    uint16_t value[BSP_LINE_SENSOR_MAX_CHANNELS];
    uint32_t valid_mask;
} BSP_LineSensor_RawSample;

/* One handle represents one ordered left-to-right sensor array. GPIO/ADC
 * instances, pins, acquisition timing and electrical scaling remain private
 * to the selected MCU BSP and its SysConfig file. */
size_t BSP_LineSensor_Count(void);
BSP_LineSensor_Handle BSP_LineSensor_GetHandle(size_t index);

/* Digital channels return 0 or 10000. Analog channels are normalized by the
 * BSP to 0..10000. Device code applies line polarity, thresholds and weights. */
bool BSP_LineSensor_GetInfo(
    BSP_LineSensor_Handle sensor, BSP_LineSensor_Info *info);
bool BSP_LineSensor_Read(
    BSP_LineSensor_Handle sensor, BSP_LineSensor_RawSample *sample);

#endif
