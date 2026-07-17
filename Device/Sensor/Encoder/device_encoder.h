#ifndef DEVICE_ENCODER_DEVICE_ENCODER_H
#define DEVICE_ENCODER_DEVICE_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_encoder.h"

typedef struct {
    BSP_Encoder_Handle backend;
    float counts_per_revolution;
    float distance_per_revolution;
    int8_t direction;
} Device_Encoder_Config;

typedef struct {
    int64_t cumulative_counts;
    int64_t delta_counts;
    float cumulative_revolutions;
    float delta_revolutions;
    float cumulative_distance;
    float delta_distance;
    float speed_revolutions_per_second;
    float speed_distance_per_second;
} Device_Encoder_Sample;

typedef struct {
    Device_Encoder_Config config;
    Device_Encoder_Sample sample;
    int64_t zero_counts;
    bool initialized;
} Device_Encoder;

bool Device_Encoder_MakeConfig(
    size_t input_index,
    float counts_per_revolution,
    float distance_per_revolution,
    int8_t direction,
    Device_Encoder_Config *config);

/* Init claims the BSP delta consumer and discards counts accumulated before
 * initialization. Use exactly one Device_Encoder per BSP encoder handle. */
bool Device_Encoder_Init(
    Device_Encoder *encoder, const Device_Encoder_Config *config);
bool Device_Encoder_Update(Device_Encoder *encoder, float dt_seconds);
bool Device_Encoder_GetSample(
    const Device_Encoder *encoder, Device_Encoder_Sample *sample);

#endif
