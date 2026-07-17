#include "device_encoder.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

static bool Device_Encoder_ApplyDirection(
    int64_t input, int8_t direction, int64_t *output)
{
    if ((output == NULL) ||
        ((direction != (int8_t)1) && (direction != (int8_t)-1)) ||
        ((direction < 0) && (input == INT64_MIN))) {
        return false;
    }
    *output = (direction > 0) ? input : -input;
    return true;
}

static bool Device_Encoder_SubtractZero(
    int64_t input, int64_t zero, int64_t *output)
{
    if ((output == NULL) ||
        ((zero > 0) && (input < INT64_MIN + zero)) ||
        ((zero < 0) && (input > INT64_MAX + zero))) {
        return false;
    }
    *output = input - zero;
    return true;
}

static bool Device_Encoder_ValidConfig(
    const Device_Encoder_Config *config)
{
    return (config != NULL) && (config->backend != NULL) &&
        isfinite(config->counts_per_revolution) &&
        isfinite(config->distance_per_revolution) &&
        (config->counts_per_revolution > 0.0F) &&
        (config->distance_per_revolution >= 0.0F) &&
        ((config->direction == (int8_t)1) ||
            (config->direction == (int8_t)-1));
}

bool Device_Encoder_MakeConfig(
    size_t input_index,
    float counts_per_revolution,
    float distance_per_revolution,
    int8_t direction,
    Device_Encoder_Config *config)
{
    Device_Encoder_Config next = {
        .backend = BSP_Encoder_GetHandle(input_index),
        .counts_per_revolution = counts_per_revolution,
        .distance_per_revolution = distance_per_revolution,
        .direction = direction,
    };

    if ((config == NULL) || !Device_Encoder_ValidConfig(&next)) {
        return false;
    }
    *config = next;
    return true;
}

static bool Device_Encoder_MakeSample(
    const Device_Encoder_Config *config,
    int64_t cumulative,
    int64_t delta,
    float dt_seconds,
    Device_Encoder_Sample *sample)
{
    float cumulative_revolutions;
    float delta_revolutions;

    cumulative_revolutions = (float)cumulative / config->counts_per_revolution;
    delta_revolutions = (float)delta / config->counts_per_revolution;
    *sample = (Device_Encoder_Sample){
        .cumulative_counts = cumulative,
        .delta_counts = delta,
        .cumulative_revolutions = cumulative_revolutions,
        .delta_revolutions = delta_revolutions,
        .cumulative_distance =
            cumulative_revolutions * config->distance_per_revolution,
        .delta_distance =
            delta_revolutions * config->distance_per_revolution,
        .speed_revolutions_per_second = delta_revolutions / dt_seconds,
        .speed_distance_per_second =
            delta_revolutions * config->distance_per_revolution / dt_seconds,
    };
    return isfinite(sample->cumulative_revolutions) &&
           isfinite(sample->delta_revolutions) &&
           isfinite(sample->cumulative_distance) &&
           isfinite(sample->delta_distance) &&
           isfinite(sample->speed_revolutions_per_second) &&
           isfinite(sample->speed_distance_per_second);
}

bool Device_Encoder_Init(
    Device_Encoder *encoder, const Device_Encoder_Config *config)
{
    Device_Encoder next = {0};
    Device_Encoder_Sample sample;
    int64_t discarded_delta;
    int64_t raw_cumulative;
    int64_t absolute_counts;

    if (encoder != NULL) {
        *encoder = (Device_Encoder){0};
    }
    if ((encoder == NULL) || !Device_Encoder_ValidConfig(config) ||
        !BSP_Encoder_GetSnapshot(config->backend, &raw_cumulative) ||
        !Device_Encoder_ApplyDirection(
            raw_cumulative, config->direction, &absolute_counts) ||
        !Device_Encoder_MakeSample(
            config, 0, 0, 1.0F, &sample) ||
        !BSP_Encoder_ReadDelta(config->backend, &discarded_delta)) {
        return false;
    }

    next.config = *config;
    next.sample = sample;
    next.zero_counts = absolute_counts;
    next.initialized = true;
    *encoder = next;
    return true;
}

bool Device_Encoder_Update(Device_Encoder *encoder, float dt_seconds)
{
    int64_t raw_delta;
    int64_t raw_cumulative;
    int64_t delta;
    int64_t absolute_counts;
    int64_t cumulative;
    Device_Encoder_Sample next;

    if ((encoder == NULL) || !encoder->initialized ||
        !Device_Encoder_ValidConfig(&encoder->config) ||
        !isfinite(dt_seconds) || (dt_seconds <= 0.0F) ||
        !BSP_Encoder_GetSnapshot(
            encoder->config.backend, &raw_cumulative) ||
        !Device_Encoder_ApplyDirection(
            raw_cumulative, encoder->config.direction, &absolute_counts) ||
        !Device_Encoder_SubtractZero(
            absolute_counts, encoder->zero_counts, &cumulative) ||
        !BSP_Encoder_ReadDelta(encoder->config.backend, &raw_delta) ||
        !Device_Encoder_ApplyDirection(
            raw_delta, encoder->config.direction, &delta) ||
        !Device_Encoder_MakeSample(
            &encoder->config, cumulative, delta, dt_seconds, &next)) {
        return false;
    }

    encoder->sample = next;
    return true;
}

bool Device_Encoder_GetSample(
    const Device_Encoder *encoder, Device_Encoder_Sample *sample)
{
    if ((encoder == NULL) || !encoder->initialized || (sample == NULL)) {
        return false;
    }
    *sample = encoder->sample;
    return true;
}
