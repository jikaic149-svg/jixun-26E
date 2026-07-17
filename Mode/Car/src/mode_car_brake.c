#include "mode_car_internal.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

Mode_Car_BrakeConfig Mode_Car_Brake_DefaultConfig(void)
{
    const Mode_Car_BrakeConfig config = {
        .stopped_speed_threshold = 10.0F,
        .stable_time_seconds = 0.025F,
        .timeout_ms = 500U,
    };
    return config;
}

bool Mode_Car_Brake_Init(
    Mode_Car_BrakeContext *context,
    const Mode_Car_BrakeConfig *config)
{
    if (context != NULL) {
        context->initialized = false;
    }
    if ((context == NULL) || (config == NULL) ||
        !Mode_Car_Internal_IsFinite(config->stopped_speed_threshold) ||
        !Mode_Car_Internal_IsFinite(config->stable_time_seconds) ||
        (config->stopped_speed_threshold < 0.0F) ||
        (config->stable_time_seconds <= 0.0F) ||
        (config->timeout_ms == 0U) ||
        (config->timeout_ms > UINT32_C(0x7fffffff))) {
        return false;
    }

    context->config = *config;
    context->started_at_ms = 0U;
    context->stable_elapsed_seconds = 0.0F;
    context->completion = MODE_CAR_COMPLETION_NONE;
    context->timing_started = false;
    context->initialized = true;
    return true;
}

bool Mode_Car_Brake_Update(
    Mode_Car_BrakeContext *context,
    const Mode_Car_BrakeInput *input,
    Mode_Car_Output *output)
{
    uint32_t started_at_ms;
    float stable_elapsed;
    Mode_Car_Completion completion;
    bool timing_started;

    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    if ((output == NULL) || (context == NULL) || !context->initialized ||
        (input == NULL) ||
        !Mode_Car_Internal_ValidTime(&input->time) ||
        !Mode_Car_Internal_IsFinite(input->measured_left_speed) ||
        !Mode_Car_Internal_IsFinite(input->measured_right_speed)) {
        return false;
    }

    if (context->completion != MODE_CAR_COMPLETION_NONE) {
        Mode_Car_Internal_StopOutput(output, context->completion);
        return true;
    }

    started_at_ms = context->started_at_ms;
    stable_elapsed = context->stable_elapsed_seconds;
    completion = context->completion;
    timing_started = context->timing_started;
    if (!timing_started) {
        started_at_ms = input->time.now_ms;
        timing_started = true;
    }

    if ((fabsf(input->measured_left_speed) <
            context->config.stopped_speed_threshold) &&
        (fabsf(input->measured_right_speed) <
            context->config.stopped_speed_threshold)) {
        stable_elapsed += input->time.dt_seconds;
        if (!Mode_Car_Internal_IsFinite(stable_elapsed)) {
            return false;
        }
    }
    else {
        stable_elapsed = 0.0F;
    }

    if ((stable_elapsed >= context->config.stable_time_seconds) ||
        ((context->config.stable_time_seconds - stable_elapsed) <=
            (8.0F * FLT_EPSILON *
                context->config.stable_time_seconds))) {
        completion = MODE_CAR_COMPLETION_STOP_STABLE;
    }
    else if ((uint32_t)(input->time.now_ms - started_at_ms) >=
        context->config.timeout_ms) {
        completion = MODE_CAR_COMPLETION_TIMEOUT;
    }

    context->started_at_ms = started_at_ms;
    context->stable_elapsed_seconds = stable_elapsed;
    context->completion = completion;
    context->timing_started = timing_started;
    Mode_Car_Internal_StopOutput(output, completion);
    return true;
}
