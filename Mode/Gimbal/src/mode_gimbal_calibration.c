#include "mode_gimbal_internal.h"

#include <math.h>
#include <stddef.h>

#define MODE_GIMBAL_DEGREES_TO_RADIANS 0.01745329251994329577F

static bool valid_feedforward_config(
    const Mode_Gimbal_GravityFeedforwardConfig *config)
{
    return (config != NULL) &&
        Mode_Gimbal_Internal_IsFinite(config->sin_coefficient) &&
        Mode_Gimbal_Internal_IsFinite(config->cos_coefficient) &&
        Mode_Gimbal_Internal_IsFinite(config->offset) &&
        Mode_Gimbal_Internal_IsFinite(config->maximum_current) &&
        (config->maximum_current > 0.0F);
}

static bool valid_gravity_config(
    const Mode_Gimbal_GravityCalibConfig *config)
{
    float usable_span;

    if ((config == NULL) ||
        !Mode_Gimbal_Internal_ValidAngleConfig(&config->pitch_loop) ||
        !Mode_Gimbal_Internal_ValidAngleConfig(&config->yaw_loop) ||
        !Mode_Gimbal_Internal_IsFinite(config->pitch_minimum_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->pitch_maximum_deg) ||
        !Mode_Gimbal_Internal_IsFinite(config->endpoint_margin_deg) ||
        (config->pitch_minimum_deg >= config->pitch_maximum_deg) ||
        (config->endpoint_margin_deg < 0.0F) ||
        (config->settle_time_ms == 0U) || (config->sample_time_ms == 0U) ||
        !valid_feedforward_config(&config->feedforward)) {
        return false;
    }
    usable_span = (config->pitch_maximum_deg - config->pitch_minimum_deg) -
        (2.0F * config->endpoint_margin_deg);
    return usable_span > 0.0F;
}

static void gravity_stop_output(Mode_Gimbal_GravityCalibOutput *output)
{
    if (output == NULL) {
        return;
    }
    Mode_Gimbal_Internal_StopIntent(&output->current);
    output->status = MODE_GIMBAL_GRAVITY_WAITING_FEEDBACK;
    output->sample_count = 0U;
    output->active_sample_index = 0U;
    output->target_pitch_deg = 0.0F;
    output->sample_ready = false;
    output->sample.encoder_angle_deg = 0.0F;
    output->sample.measured_current = 0.0F;
}

static float gravity_target_for_index(
    const Mode_Gimbal_GravityCalibContext *context, size_t sample_index)
{
    size_t point_index;
    float low = context->config.pitch_minimum_deg +
        context->config.endpoint_margin_deg;
    float high = context->config.pitch_maximum_deg -
        context->config.endpoint_margin_deg;

    if (sample_index < MODE_GIMBAL_GRAVITY_POINTS_PER_PASS) {
        point_index = sample_index;
    }
    else {
        point_index = (MODE_GIMBAL_GRAVITY_SAMPLE_CAPACITY - 1U) -
            sample_index;
    }
    return low + ((high - low) * (float) point_index /
        (float) (MODE_GIMBAL_GRAVITY_POINTS_PER_PASS - 1U));
}

Mode_Gimbal_GravityFeedforwardConfig
Mode_Gimbal_GravityFeedforward_DefaultConfig(void)
{
    Mode_Gimbal_GravityFeedforwardConfig config;

    config.sin_coefficient = 0.0300124F;
    config.cos_coefficient = 0.0200932F;
    config.offset = 0.0038665F;
    config.maximum_current = 0.6F;
    return config;
}

bool Mode_Gimbal_GravityFeedforward_Calculate(
    const Mode_Gimbal_GravityFeedforwardConfig *config,
    float pitch_encoder_deg,
    float *current_intent)
{
    float radians;
    float current;

    if (!valid_feedforward_config(config) || (current_intent == NULL) ||
        !Mode_Gimbal_Internal_IsFinite(pitch_encoder_deg)) {
        return false;
    }
    radians = pitch_encoder_deg * MODE_GIMBAL_DEGREES_TO_RADIANS;
    current = (config->sin_coefficient * sinf(radians)) +
        (config->cos_coefficient * cosf(radians)) + config->offset;
    if (!Mode_Gimbal_Internal_IsFinite(current)) {
        return false;
    }
    *current_intent = Mode_Gimbal_Internal_Clamp(current,
        -config->maximum_current, config->maximum_current);
    return true;
}

Mode_Gimbal_GravityCalibConfig Mode_Gimbal_GravityCalib_DefaultConfig(void)
{
    Mode_Gimbal_GravityCalibConfig config;

    config.pitch_loop = Mode_Gimbal_AngleLoop_DefaultPitchConfig();
    config.yaw_loop = Mode_Gimbal_AngleLoop_DefaultYawConfig();
    config.pitch_minimum_deg = 19.0F;
    config.pitch_maximum_deg = 112.0F;
    config.endpoint_margin_deg = 6.0F;
    config.settle_time_ms = 1200U;
    config.sample_time_ms = 900U;
    config.feedforward = Mode_Gimbal_GravityFeedforward_DefaultConfig();
    return config;
}

bool Mode_Gimbal_GravityCalib_Init(
    Mode_Gimbal_GravityCalibContext *context,
    const Mode_Gimbal_GravityCalibConfig *config)
{
    Mode_Gimbal_AngleLoopContext pitch_loop;
    Mode_Gimbal_AngleLoopContext yaw_loop;
    size_t index;

    if ((context == NULL) || !valid_gravity_config(config) ||
        !Mode_Gimbal_AngleLoop_Init(&pitch_loop, &config->pitch_loop) ||
        !Mode_Gimbal_AngleLoop_Init(&yaw_loop, &config->yaw_loop)) {
        return false;
    }

    context->config = *config;
    context->pitch_loop = pitch_loop;
    context->yaw_loop = yaw_loop;
    for (index = 0U; index < MODE_GIMBAL_GRAVITY_SAMPLE_CAPACITY; ++index) {
        context->samples[index].encoder_angle_deg = 0.0F;
        context->samples[index].measured_current = 0.0F;
    }
    context->sample_count = 0U;
    context->phase_started_at_ms = 0U;
    context->yaw_target_deg = 0.0F;
    context->accumulated_angle_seconds = 0.0F;
    context->accumulated_current_seconds = 0.0F;
    context->accumulated_seconds = 0.0F;
    context->phase = MODE_GIMBAL_GRAVITY_WAITING_FEEDBACK;
    context->target_initialized = false;
    context->phase_timing_started = false;
    context->initialized = true;
    return true;
}

bool Mode_Gimbal_GravityCalib_Update(
    Mode_Gimbal_GravityCalibContext *context,
    const Mode_Gimbal_GravityCalibInput *input,
    Mode_Gimbal_GravityCalibOutput *output)
{
    Mode_Gimbal_GravityCalibContext next;
    Mode_Gimbal_AngleLoopInput loop_input;
    Mode_Gimbal_AngleLoopOutput pitch_output;
    Mode_Gimbal_AngleLoopOutput yaw_output;
    float target_pitch;

    gravity_stop_output(output);
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized ||
        !Mode_Gimbal_Internal_ValidTime(&input->time)) {
        return false;
    }
    next = *context;
    output->sample_count = next.sample_count;
    output->active_sample_index = next.sample_count;

    if (next.phase == MODE_GIMBAL_GRAVITY_COMPLETE) {
        output->status = MODE_GIMBAL_GRAVITY_COMPLETE;
        return true;
    }
    if (!input->encoder_feedback_valid) {
        next.phase_timing_started = false;
        if (next.phase == MODE_GIMBAL_GRAVITY_SAMPLING) {
            next.accumulated_angle_seconds = 0.0F;
            next.accumulated_current_seconds = 0.0F;
            next.accumulated_seconds = 0.0F;
        }
        output->status = MODE_GIMBAL_GRAVITY_WAITING_FEEDBACK;
        *context = next;
        return true;
    }
    if (!Mode_Gimbal_Internal_IsFinite(input->pitch_encoder_deg) ||
        !Mode_Gimbal_Internal_IsFinite(input->yaw_encoder_deg) ||
        !Mode_Gimbal_Internal_IsFinite(
            input->pitch_motor_speed_deg_per_second) ||
        !Mode_Gimbal_Internal_IsFinite(
            input->yaw_motor_speed_deg_per_second) ||
        !Mode_Gimbal_Internal_IsFinite(input->measured_pitch_current)) {
        return false;
    }

    if (!next.target_initialized) {
        next.yaw_target_deg = input->yaw_encoder_deg;
        next.target_initialized = true;
        next.phase = MODE_GIMBAL_GRAVITY_SETTLING;
        next.phase_started_at_ms = input->time.now_ms;
        next.phase_timing_started = true;
    }
    if (!next.phase_timing_started) {
        next.phase_started_at_ms = input->time.now_ms;
        next.phase_timing_started = true;
    }

    target_pitch = gravity_target_for_index(&next, next.sample_count);
    loop_input.dt_seconds = input->time.dt_seconds;
    loop_input.target_angle_deg = target_pitch;
    loop_input.measured_angle_deg = input->pitch_encoder_deg;
    loop_input.measured_speed_deg_per_second =
        input->pitch_motor_speed_deg_per_second;
    if (!Mode_Gimbal_AngleLoop_Update(
            &next.pitch_loop, &loop_input, &pitch_output)) {
        return false;
    }
    loop_input.target_angle_deg = next.yaw_target_deg;
    loop_input.measured_angle_deg = input->yaw_encoder_deg;
    loop_input.measured_speed_deg_per_second =
        input->yaw_motor_speed_deg_per_second;
    if (!Mode_Gimbal_AngleLoop_Update(
            &next.yaw_loop, &loop_input, &yaw_output)) {
        return false;
    }

    output->current.pitch_current = pitch_output.current_intent;
    output->current.yaw_current = yaw_output.current_intent;
    output->current.current_requested = true;
    output->target_pitch_deg = target_pitch;
    output->sample_count = next.sample_count;
    output->active_sample_index = next.sample_count;

    if (next.phase == MODE_GIMBAL_GRAVITY_SETTLING) {
        output->status = MODE_GIMBAL_GRAVITY_SETTLING;
        if (Mode_Gimbal_Internal_ElapsedMs(input->time.now_ms,
                next.phase_started_at_ms) >= next.config.settle_time_ms) {
            next.phase = MODE_GIMBAL_GRAVITY_SAMPLING;
            next.phase_started_at_ms = input->time.now_ms;
            next.accumulated_angle_seconds = 0.0F;
            next.accumulated_current_seconds = 0.0F;
            next.accumulated_seconds = 0.0F;
            output->status = MODE_GIMBAL_GRAVITY_SAMPLING;
        }
    }
    else if (next.phase == MODE_GIMBAL_GRAVITY_SAMPLING) {
        next.accumulated_angle_seconds +=
            input->pitch_encoder_deg * input->time.dt_seconds;
        next.accumulated_current_seconds +=
            input->measured_pitch_current * input->time.dt_seconds;
        next.accumulated_seconds += input->time.dt_seconds;
        output->status = MODE_GIMBAL_GRAVITY_SAMPLING;

        if (Mode_Gimbal_Internal_ElapsedMs(input->time.now_ms,
                next.phase_started_at_ms) >= next.config.sample_time_ms) {
            Mode_Gimbal_GravitySample completed;

            if ((next.accumulated_seconds <= 0.0F) ||
                !Mode_Gimbal_Internal_IsFinite(next.accumulated_seconds)) {
                return false;
            }
            completed.encoder_angle_deg = next.accumulated_angle_seconds /
                next.accumulated_seconds;
            completed.measured_current = next.accumulated_current_seconds /
                next.accumulated_seconds;
            if (!Mode_Gimbal_Internal_IsFinite(completed.encoder_angle_deg) ||
                !Mode_Gimbal_Internal_IsFinite(completed.measured_current)) {
                return false;
            }
            next.samples[next.sample_count] = completed;
            ++next.sample_count;
            output->sample_ready = true;
            output->sample = completed;
            output->sample_count = next.sample_count;
            output->active_sample_index = next.sample_count - 1U;
            next.accumulated_angle_seconds = 0.0F;
            next.accumulated_current_seconds = 0.0F;
            next.accumulated_seconds = 0.0F;

            if (next.sample_count >=
                MODE_GIMBAL_GRAVITY_SAMPLE_CAPACITY) {
                next.phase = MODE_GIMBAL_GRAVITY_COMPLETE;
                output->status = MODE_GIMBAL_GRAVITY_COMPLETE;
                Mode_Gimbal_Internal_StopIntent(&output->current);
            }
            else {
                next.phase = MODE_GIMBAL_GRAVITY_SETTLING;
                next.phase_started_at_ms = input->time.now_ms;
                output->status = MODE_GIMBAL_GRAVITY_SAMPLE_READY;
            }
        }
    }
    else {
        return false;
    }

    *context = next;
    return true;
}

size_t Mode_Gimbal_GravityCalib_SampleCount(
    const Mode_Gimbal_GravityCalibContext *context)
{
    return ((context != NULL) && context->initialized) ?
        context->sample_count : 0U;
}

bool Mode_Gimbal_GravityCalib_GetSample(
    const Mode_Gimbal_GravityCalibContext *context,
    size_t index,
    Mode_Gimbal_GravitySample *sample)
{
    if ((context == NULL) || (sample == NULL) || !context->initialized ||
        (index >= context->sample_count)) {
        return false;
    }
    *sample = context->samples[index];
    return true;
}
