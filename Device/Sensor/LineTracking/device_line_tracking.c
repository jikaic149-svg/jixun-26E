#include "device_line_tracking.h"

#include <math.h>
#include <stddef.h>

static bool finite_value(float value)
{
    return isfinite(value) != 0;
}

static bool valid_config(const Device_LineTracking_Config *config)
{
    BSP_LineSensor_Info info;
    size_t index;

    if ((config == NULL) || (config->backend == NULL) ||
        !BSP_LineSensor_GetInfo(config->backend, &info) ||
        (config->channel_count == 0U) ||
        (config->channel_count > BSP_LINE_SENSOR_MAX_CHANNELS) ||
        (config->channel_count != info.channel_count) ||
        !finite_value(config->target_position) ||
        !finite_value(config->minimum_strength_sum) ||
        (config->minimum_strength_sum < 0.0F) ||
        (config->minimum_strength_sum > (float)config->channel_count) ||
        (config->intersection_minimum_active_channels >
            config->channel_count)) {
        return false;
    }

    for (index = 0U; index < config->channel_count; ++index) {
        const Device_LineTracking_ChannelConfig *channel =
            &config->channel[index];
        if ((channel->calibration_max > BSP_LINE_SENSOR_VALUE_MAX) ||
            (channel->calibration_min >= channel->calibration_max) ||
            (channel->threshold >= BSP_LINE_SENSOR_VALUE_MAX) ||
            !finite_value(channel->position_weight)) {
            return false;
        }
    }
    return true;
}

bool Device_LineTracking_DefaultConfig(
    BSP_LineSensor_Handle backend, Device_LineTracking_Config *config)
{
    BSP_LineSensor_Info info;
    Device_LineTracking_Config next = {0};
    size_t index;

    if ((config == NULL) || (backend == NULL) ||
        !BSP_LineSensor_GetInfo(backend, &info) ||
        (info.channel_count == 0U) ||
        (info.channel_count > BSP_LINE_SENSOR_MAX_CHANNELS)) {
        return false;
    }

    next.backend = backend;
    next.channel_count = info.channel_count;
    next.target_position = 0.0F;
    next.minimum_strength_sum = 0.05F;
    next.intersection_minimum_active_channels =
        (info.channel_count >= 3U) ? 3U : info.channel_count;
    next.estimator = Algorithm_LineTracking_DefaultConfig();
    for (index = 0U; index < info.channel_count; ++index) {
        next.channel[index].calibration_min = 0U;
        next.channel[index].calibration_max =
            BSP_LINE_SENSOR_VALUE_MAX;
        next.channel[index].threshold = 5000U;
        next.channel[index].line_is_high = true;
        next.channel[index].position_weight =
            (info.channel_count == 1U) ? 0.0F :
            -1.0F + (2.0F * (float)index /
                (float)(info.channel_count - 1U));
    }
    *config = next;
    return true;
}

bool Device_LineTracking_DefaultConfigByIndex(
    size_t input_index, Device_LineTracking_Config *config)
{
    return Device_LineTracking_DefaultConfig(
        BSP_LineSensor_GetHandle(input_index), config);
}

bool Device_LineTracking_Init(
    Device_LineTracking *device,
    const Device_LineTracking_Config *config)
{
    Device_LineTracking next = {0};

    if (device != NULL) {
        *device = (Device_LineTracking){0};
    }
    if ((device == NULL) || !valid_config(config) ||
        !Algorithm_LineTracking_Init(&next.estimator, &config->estimator)) {
        return false;
    }

    next.config = *config;
    next.sample.channel_count = config->channel_count;
    next.sample.centroid_position = config->target_position;
    next.sample.difference_position = config->target_position;
    next.sample.raw_position = config->target_position;
    next.sample.position = config->target_position;
    next.sample.error = 0.0F;
    next.sample.line_lost = true;
    next.initialized = true;
    *device = next;
    return true;
}

bool Device_LineTracking_Update(
    Device_LineTracking *device, uint32_t now_ms)
{
    BSP_LineSensor_RawSample raw;
    Device_LineTracking_Sample next = {0};
    Algorithm_LineTracking_Input estimator_input;
    Algorithm_LineTracking_Output estimator_output;
    float weights[BSP_LINE_SENSOR_MAX_CHANNELS];
    size_t index;

    if ((device == NULL) || !device->initialized ||
        !valid_config(&device->config) ||
        !BSP_LineSensor_Read(device->config.backend, &raw)) {
        return false;
    }

    next.channel_count = device->config.channel_count;
    next.sampled_at_ms = now_ms;
    for (index = 0U; index < next.channel_count; ++index) {
        const uint32_t bit = UINT32_C(1) << index;
        const Device_LineTracking_ChannelConfig *channel =
            &device->config.channel[index];
        float response;
        float threshold;
        float strength;

        if (((raw.valid_mask & bit) == 0U) ||
            (raw.value[index] > BSP_LINE_SENSOR_VALUE_MAX)) {
            return false;
        }
        if (raw.value[index] != BSP_LINE_SENSOR_VALUE_MIN) {
            next.raw_high_mask |= bit;
        }
        response = ((float)raw.value[index] -
            (float)channel->calibration_min) /
            ((float)channel->calibration_max -
                (float)channel->calibration_min);
        if (response < 0.0F) {
            response = 0.0F;
        }
        else if (response > 1.0F) {
            response = 1.0F;
        }
        if (!channel->line_is_high) {
            response = 1.0F - response;
        }
        threshold = (float)channel->threshold /
            (float)BSP_LINE_SENSOR_VALUE_MAX;
        strength = (response > threshold) ?
            (response - threshold) / (1.0F - threshold) : 0.0F;
        if (!finite_value(strength)) {
            return false;
        }

        next.normalized[index] = strength;
        weights[index] = channel->position_weight;
        next.strength_sum += strength;
        if (strength > 0.0F) {
            next.active_mask |= bit;
            ++next.active_count;
        }
    }

    next.line_present = (next.active_count > 0U) &&
        (next.strength_sum >= device->config.minimum_strength_sum);
    next.line_lost = !next.line_present;
    next.intersection = next.line_present &&
        (device->config.intersection_minimum_active_channels > 0U) &&
        (next.active_count >=
            device->config.intersection_minimum_active_channels);

    if (next.line_present) {
        estimator_input.strength = next.normalized;
        estimator_input.position_weight = weights;
        estimator_input.channel_count = next.channel_count;
        estimator_input.target_position = device->config.target_position;
        estimator_input.dt_seconds = device->has_previous_sample_time ?
            (float)((uint32_t)(now_ms - device->previous_sample_ms)) /
                1000.0F : 0.0F;
        if (!Algorithm_LineTracking_Update(
                &device->estimator, &estimator_input, &estimator_output)) {
            return false;
        }
        next.centroid_position = estimator_output.centroid_position;
        next.difference_ratio = estimator_output.difference_ratio;
        next.difference_position = estimator_output.difference_position;
        next.raw_position = estimator_output.raw_position;
        next.position = estimator_output.filtered_position;
        next.confidence = estimator_output.confidence;
        device->last_valid_position = next.position;
        device->has_last_valid_position = true;
    }
    else {
        Algorithm_LineTracking_Reset(&device->estimator);
        next.centroid_position = device->config.target_position;
        next.difference_position = device->config.target_position;
        next.raw_position = device->has_last_valid_position ?
            device->last_valid_position : device->config.target_position;
        next.position = next.raw_position;
    }
    next.error = next.position - device->config.target_position;
    if (!finite_value(next.error)) {
        return false;
    }

    device->previous_sample_ms = now_ms;
    device->has_previous_sample_time = true;
    device->sample = next;
    return true;
}

bool Device_LineTracking_GetSample(
    const Device_LineTracking *device,
    Device_LineTracking_Sample *sample)
{
    if ((device == NULL) || !device->initialized || (sample == NULL)) {
        return false;
    }
    *sample = device->sample;
    return true;
}
