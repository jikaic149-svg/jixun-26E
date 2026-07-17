#include "algorithm_line_tracking.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

static bool finite_value(float value)
{
    return isfinite(value) != 0;
}

static float clamp_value(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

Algorithm_LineTracking_Config Algorithm_LineTracking_DefaultConfig(void)
{
    const Algorithm_LineTracking_Config config = {
        .denominator_epsilon = 0.02F,
        .difference_weight = 0.70F,
        .low_pass_time_constant_seconds = 0.015F,
    };
    return config;
}

bool Algorithm_LineTracking_Init(
    Algorithm_LineTracking *algorithm,
    const Algorithm_LineTracking_Config *config)
{
    if (algorithm != NULL) {
        *algorithm = (Algorithm_LineTracking){0};
    }
    if ((algorithm == NULL) || (config == NULL) ||
        !finite_value(config->denominator_epsilon) ||
        !finite_value(config->difference_weight) ||
        !finite_value(config->low_pass_time_constant_seconds) ||
        (config->denominator_epsilon <= 0.0F) ||
        (config->difference_weight < 0.0F) ||
        (config->difference_weight > 1.0F) ||
        (config->low_pass_time_constant_seconds < 0.0F)) {
        return false;
    }

    algorithm->config = *config;
    algorithm->initialized = true;
    return true;
}

void Algorithm_LineTracking_Reset(Algorithm_LineTracking *algorithm)
{
    if ((algorithm == NULL) || !algorithm->initialized) {
        return;
    }
    algorithm->filtered_position = 0.0F;
    algorithm->filter_initialized = false;
}

bool Algorithm_LineTracking_Update(
    Algorithm_LineTracking *algorithm,
    const Algorithm_LineTracking_Input *input,
    Algorithm_LineTracking_Output *output)
{
    Algorithm_LineTracking_Output next = {0};
    float weighted_sum = 0.0F;
    float strength_sum = 0.0F;
    float minimum_position = FLT_MAX;
    float maximum_position = -FLT_MAX;
    float left_sum = 0.0F;
    float right_sum = 0.0F;
    float maximum_span = 0.0F;
    size_t left_count = 0U;
    size_t right_count = 0U;
    size_t index;

    if ((algorithm == NULL) || !algorithm->initialized ||
        (input == NULL) || (output == NULL) ||
        (input->strength == NULL) || (input->position_weight == NULL) ||
        (input->channel_count == 0U) ||
        !finite_value(input->target_position) ||
        !finite_value(input->dt_seconds) || (input->dt_seconds < 0.0F)) {
        return false;
    }

    for (index = 0U; index < input->channel_count; ++index) {
        const float strength = input->strength[index];
        const float position = input->position_weight[index];
        float span;

        if (!finite_value(strength) || !finite_value(position) ||
            (strength < 0.0F) || (strength > 1.0F)) {
            return false;
        }
        strength_sum += strength;
        weighted_sum += strength * position;
        if (position < minimum_position) {
            minimum_position = position;
        }
        if (position > maximum_position) {
            maximum_position = position;
        }
        span = fabsf(position - input->target_position);
        if (span > maximum_span) {
            maximum_span = span;
        }
        if (position < input->target_position) {
            left_sum += strength;
            ++left_count;
        }
        else if (position > input->target_position) {
            right_sum += strength;
            ++right_count;
        }
    }
    if (!finite_value(strength_sum) ||
        (strength_sum <= algorithm->config.denominator_epsilon)) {
        return false;
    }

    next.centroid_position = weighted_sum / strength_sum;
    next.left_level = (left_count > 0U) ?
        left_sum / (float)left_count : 0.0F;
    next.right_level = (right_count > 0U) ?
        right_sum / (float)right_count : 0.0F;
    next.denominator = next.left_level + next.right_level;
    next.difference_ratio =
        (next.right_level - next.left_level) /
        (next.denominator + algorithm->config.denominator_epsilon);
    next.difference_ratio = clamp_value(next.difference_ratio, -1.0F, 1.0F);
    next.difference_position = input->target_position +
        (next.difference_ratio * maximum_span);
    next.raw_position =
        (algorithm->config.difference_weight * next.difference_position) +
        ((1.0F - algorithm->config.difference_weight) *
            next.centroid_position);
    next.raw_position = clamp_value(
        next.raw_position, minimum_position, maximum_position);
    next.confidence = next.denominator /
        (next.denominator + algorithm->config.denominator_epsilon);

    if (!algorithm->filter_initialized ||
        (algorithm->config.low_pass_time_constant_seconds == 0.0F)) {
        next.filtered_position = next.raw_position;
    }
    else if (input->dt_seconds == 0.0F) {
        next.filtered_position = algorithm->filtered_position;
    }
    else {
        const float alpha = input->dt_seconds /
            (algorithm->config.low_pass_time_constant_seconds +
                input->dt_seconds);
        next.filtered_position = algorithm->filtered_position +
            (alpha * (next.raw_position - algorithm->filtered_position));
    }

    if (!finite_value(next.centroid_position) ||
        !finite_value(next.difference_ratio) ||
        !finite_value(next.difference_position) ||
        !finite_value(next.raw_position) ||
        !finite_value(next.filtered_position) ||
        !finite_value(next.confidence)) {
        return false;
    }

    algorithm->filtered_position = next.filtered_position;
    algorithm->filter_initialized = true;
    *output = next;
    return true;
}
