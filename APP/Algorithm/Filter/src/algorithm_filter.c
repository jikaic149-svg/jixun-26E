#include "algorithm_filter.h"

#include <math.h>
#include <stddef.h>

bool Algorithm_Filter_LowPass_Init(
    Algorithm_Filter_LowPass *filter,
    float time_constant_seconds,
    float initial_value)
{
    if ((filter == NULL) || !isfinite(time_constant_seconds) ||
        !isfinite(initial_value) || (time_constant_seconds < 0.0F)) {
        return false;
    }
    filter->time_constant_seconds = time_constant_seconds;
    filter->value = initial_value;
    filter->initialized = true;
    return true;
}

bool Algorithm_Filter_LowPass_Update(
    Algorithm_Filter_LowPass *filter,
    float input,
    float dt_seconds,
    float *output)
{
    float alpha;
    float next;

    if ((filter == NULL) || !filter->initialized || (output == NULL) ||
        !isfinite(filter->time_constant_seconds) ||
        (filter->time_constant_seconds < 0.0F) ||
        !isfinite(filter->value) || !isfinite(input) ||
        !isfinite(dt_seconds) || (dt_seconds <= 0.0F)) {
        return false;
    }
    alpha = (filter->time_constant_seconds == 0.0F)
        ? 1.0F
        : dt_seconds / (filter->time_constant_seconds + dt_seconds);
    next = filter->value + ((input - filter->value) * alpha);
    if (!isfinite(next)) {
        return false;
    }
    filter->value = next;
    *output = next;
    return true;
}

void Algorithm_Filter_LowPass_Reset(
    Algorithm_Filter_LowPass *filter, float value)
{
    if ((filter == NULL) || !filter->initialized || !isfinite(value)) {
        return;
    }
    filter->value = value;
}

bool Algorithm_Filter_MovingAverage_Init(
    Algorithm_Filter_MovingAverage *filter,
    float *storage,
    size_t capacity)
{
    size_t index;

    if ((filter == NULL) || (storage == NULL) || (capacity == 0U)) {
        return false;
    }
    for (index = 0U; index < capacity; ++index) {
        storage[index] = 0.0F;
    }
    *filter = (Algorithm_Filter_MovingAverage){
        .storage = storage,
        .capacity = capacity,
        .initialized = true,
    };
    return true;
}

bool Algorithm_Filter_MovingAverage_Update(
    Algorithm_Filter_MovingAverage *filter,
    float input,
    float *output)
{
    float next_sum;
    float next_output;
    size_t next_count;
    size_t next_index;

    if ((filter == NULL) || !filter->initialized || (output == NULL) ||
        (filter->storage == NULL) || (filter->capacity == 0U) ||
        (filter->count > filter->capacity) ||
        (filter->next >= filter->capacity) || !isfinite(filter->sum) ||
        !isfinite(input)) {
        return false;
    }
    next_sum = filter->sum;
    next_count = filter->count;
    if (filter->count == filter->capacity) {
        next_sum -= filter->storage[filter->next];
    }
    else {
        ++next_count;
    }
    next_sum += input;
    if (!isfinite(next_sum) || (next_count == 0U)) {
        return false;
    }
    next_output = next_sum / (float)next_count;
    if (!isfinite(next_output)) {
        return false;
    }
    next_index = (filter->next + 1U) % filter->capacity;

    filter->storage[filter->next] = input;
    filter->count = next_count;
    filter->next = next_index;
    filter->sum = next_sum;
    *output = next_output;
    return true;
}

void Algorithm_Filter_MovingAverage_Reset(
    Algorithm_Filter_MovingAverage *filter)
{
    size_t index;

    if ((filter == NULL) || !filter->initialized) {
        return;
    }
    for (index = 0U; index < filter->capacity; ++index) {
        filter->storage[index] = 0.0F;
    }
    filter->count = 0U;
    filter->next = 0U;
    filter->sum = 0.0F;
}

bool Algorithm_Filter_Median3(
    float a, float b, float c, float *output)
{
    float median;

    if ((output == NULL) || !isfinite(a) || !isfinite(b) || !isfinite(c)) {
        return false;
    }
    if (a > b) {
        median = a;
        a = b;
        b = median;
    }
    if (b > c) {
        median = b;
        b = c;
        c = median;
    }
    if (a > b) {
        b = a;
    }
    *output = b;
    return true;
}

bool Algorithm_Filter_Biquad_Init(
    Algorithm_Filter_Biquad *filter,
    Algorithm_Filter_BiquadType type,
    float sample_rate_hz,
    float cutoff_hz,
    float q)
{
    const float pi = 3.14159265358979323846F;
    float omega;
    float sine;
    float cosine;
    float alpha;
    float numerator0;
    float numerator1;
    float numerator2;
    float denominator0;
    float denominator1;
    float denominator2;
    Algorithm_Filter_Biquad next = {0};

    if (filter != NULL) {
        *filter = (Algorithm_Filter_Biquad){0};
    }
    if ((filter == NULL) || !isfinite(sample_rate_hz) ||
        !isfinite(cutoff_hz) || !isfinite(q) ||
        (sample_rate_hz <= 0.0F) || (cutoff_hz <= 0.0F) ||
        (cutoff_hz >= (sample_rate_hz * 0.5F)) || (q <= 0.0F) ||
        ((type != ALGORITHM_FILTER_BIQUAD_LOWPASS) &&
         (type != ALGORITHM_FILTER_BIQUAD_HIGHPASS) &&
         (type != ALGORITHM_FILTER_BIQUAD_BANDPASS_PEAK) &&
         (type != ALGORITHM_FILTER_BIQUAD_NOTCH))) {
        return false;
    }
    omega = 2.0F * pi * cutoff_hz / sample_rate_hz;
    sine = sinf(omega);
    cosine = cosf(omega);
    alpha = sine / (2.0F * q);
    denominator0 = 1.0F + alpha;
    denominator1 = -2.0F * cosine;
    denominator2 = 1.0F - alpha;
    switch (type) {
    case ALGORITHM_FILTER_BIQUAD_LOWPASS:
        numerator0 = (1.0F - cosine) * 0.5F;
        numerator1 = 2.0F * numerator0;
        numerator2 = numerator0;
        break;
    case ALGORITHM_FILTER_BIQUAD_HIGHPASS:
        numerator0 = (1.0F + cosine) * 0.5F;
        numerator1 = -2.0F * numerator0;
        numerator2 = numerator0;
        break;
    case ALGORITHM_FILTER_BIQUAD_BANDPASS_PEAK:
        numerator0 = alpha;
        numerator1 = 0.0F;
        numerator2 = -alpha;
        break;
    case ALGORITHM_FILTER_BIQUAD_NOTCH:
        numerator0 = 1.0F;
        numerator1 = -2.0F * cosine;
        numerator2 = 1.0F;
        break;
    default:
        return false;
    }
    next.b0 = numerator0 / denominator0;
    next.b1 = numerator1 / denominator0;
    next.b2 = numerator2 / denominator0;
    next.a1 = denominator1 / denominator0;
    next.a2 = denominator2 / denominator0;
    if (!isfinite(next.b0) || !isfinite(next.b1) ||
        !isfinite(next.b2) || !isfinite(next.a1) ||
        !isfinite(next.a2)) {
        return false;
    }
    next.initialized = true;
    *filter = next;
    return true;
}

bool Algorithm_Filter_Biquad_Update(
    Algorithm_Filter_Biquad *filter,
    float input,
    float *output)
{
    float next_output;

    if ((filter == NULL) || !filter->initialized || (output == NULL) ||
        !isfinite(input) || !isfinite(filter->x1) ||
        !isfinite(filter->x2) || !isfinite(filter->y1) ||
        !isfinite(filter->y2)) {
        return false;
    }
    next_output = (filter->b0 * input) + (filter->b1 * filter->x1) +
        (filter->b2 * filter->x2) - (filter->a1 * filter->y1) -
        (filter->a2 * filter->y2);
    if (!isfinite(next_output)) {
        return false;
    }
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = next_output;
    *output = next_output;
    return true;
}

void Algorithm_Filter_Biquad_Reset(Algorithm_Filter_Biquad *filter)
{
    if ((filter == NULL) || !filter->initialized) {
        return;
    }
    filter->x1 = 0.0F;
    filter->x2 = 0.0F;
    filter->y1 = 0.0F;
    filter->y2 = 0.0F;
}

bool Algorithm_Filter_AngleKalman_Init(
    Algorithm_Filter_AngleKalman *filter,
    float q_angle,
    float q_bias,
    float r_measure,
    float initial_angle_deg)
{
    if (filter != NULL) {
        *filter = (Algorithm_Filter_AngleKalman){0};
    }
    if ((filter == NULL) || !isfinite(q_angle) || !isfinite(q_bias) ||
        !isfinite(r_measure) || !isfinite(initial_angle_deg) ||
        (q_angle < 0.0F) || (q_bias < 0.0F) || (r_measure <= 0.0F)) {
        return false;
    }
    *filter = (Algorithm_Filter_AngleKalman){
        .q_angle = q_angle,
        .q_bias = q_bias,
        .r_measure = r_measure,
        .angle_deg = initial_angle_deg,
        .initialized = true,
    };
    return true;
}

bool Algorithm_Filter_AngleKalman_SetAngle(
    Algorithm_Filter_AngleKalman *filter, float angle_deg)
{
    if ((filter == NULL) || !filter->initialized || !isfinite(angle_deg)) {
        return false;
    }
    filter->angle_deg = angle_deg;
    return true;
}

bool Algorithm_Filter_AngleKalman_Predict(
    Algorithm_Filter_AngleKalman *filter,
    float gyro_rate_dps,
    float dt_seconds,
    float *angle_deg)
{
    Algorithm_Filter_AngleKalman next;

    if ((filter == NULL) || !filter->initialized || (angle_deg == NULL) ||
        !isfinite(gyro_rate_dps) || !isfinite(dt_seconds) ||
        (dt_seconds <= 0.0F)) {
        return false;
    }
    next = *filter;
    next.rate_dps = gyro_rate_dps - next.bias_dps;
    next.angle_deg += dt_seconds * next.rate_dps;
    next.p00 += dt_seconds * ((dt_seconds * next.p11) - next.p01 -
        next.p10 + next.q_angle);
    next.p01 -= dt_seconds * next.p11;
    next.p10 -= dt_seconds * next.p11;
    next.p11 += next.q_bias * dt_seconds;
    if (!isfinite(next.angle_deg) || !isfinite(next.rate_dps) ||
        !isfinite(next.p00) || !isfinite(next.p01) ||
        !isfinite(next.p10) || !isfinite(next.p11)) {
        return false;
    }
    *filter = next;
    *angle_deg = next.angle_deg;
    return true;
}

bool Algorithm_Filter_AngleKalman_Correct(
    Algorithm_Filter_AngleKalman *filter,
    float acceleration_angle_deg,
    float *angle_deg)
{
    Algorithm_Filter_AngleKalman next;
    float innovation_covariance;
    float gain0;
    float gain1;
    float innovation;
    float p00;
    float p01;

    if ((filter == NULL) || !filter->initialized || (angle_deg == NULL) ||
        !isfinite(acceleration_angle_deg)) {
        return false;
    }
    next = *filter;
    innovation_covariance = next.p00 + next.r_measure;
    if (!isfinite(innovation_covariance) ||
        (innovation_covariance <= 0.0F)) {
        return false;
    }
    gain0 = next.p00 / innovation_covariance;
    gain1 = next.p10 / innovation_covariance;
    innovation = acceleration_angle_deg - next.angle_deg;
    next.angle_deg += gain0 * innovation;
    next.bias_dps += gain1 * innovation;
    p00 = next.p00;
    p01 = next.p01;
    next.p00 -= gain0 * p00;
    next.p01 -= gain0 * p01;
    next.p10 -= gain1 * p00;
    next.p11 -= gain1 * p01;
    if (!isfinite(next.angle_deg) || !isfinite(next.bias_dps) ||
        !isfinite(next.p00) || !isfinite(next.p01) ||
        !isfinite(next.p10) || !isfinite(next.p11)) {
        return false;
    }
    *filter = next;
    *angle_deg = next.angle_deg;
    return true;
}

bool Algorithm_Filter_AngleKalman_Update(
    Algorithm_Filter_AngleKalman *filter,
    float acceleration_angle_deg,
    float gyro_rate_dps,
    float dt_seconds,
    float *angle_deg)
{
    Algorithm_Filter_AngleKalman next;
    float ignored;

    if ((filter == NULL) || (angle_deg == NULL)) {
        return false;
    }
    next = *filter;
    if (!Algorithm_Filter_AngleKalman_Predict(
            &next, gyro_rate_dps, dt_seconds, &ignored) ||
        !Algorithm_Filter_AngleKalman_Correct(
            &next, acceleration_angle_deg, angle_deg)) {
        return false;
    }
    *filter = next;
    return true;
}

bool Algorithm_Filter_AngleKalman_GetRate(
    const Algorithm_Filter_AngleKalman *filter, float *rate_dps)
{
    if ((filter == NULL) || !filter->initialized || (rate_dps == NULL) ||
        !isfinite(filter->rate_dps)) {
        return false;
    }
    *rate_dps = filter->rate_dps;
    return true;
}

bool Algorithm_Filter_ScalarKalman_Init(
    Algorithm_Filter_ScalarKalman *filter,
    float process_noise,
    float measurement_noise)
{
    if (filter != NULL) {
        *filter = (Algorithm_Filter_ScalarKalman){0};
    }
    if ((filter == NULL) || !isfinite(process_noise) ||
        !isfinite(measurement_noise) || (process_noise < 0.0F) ||
        (measurement_noise <= 0.0F)) {
        return false;
    }
    *filter = (Algorithm_Filter_ScalarKalman){
        .process_noise = process_noise,
        .measurement_noise = measurement_noise,
        .covariance = 1.0F,
        .initialized = true,
    };
    return true;
}

bool Algorithm_Filter_ScalarKalman_SetState(
    Algorithm_Filter_ScalarKalman *filter, float value)
{
    if ((filter == NULL) || !filter->initialized || !isfinite(value)) {
        return false;
    }
    filter->value = value;
    filter->covariance = 1.0F;
    filter->seeded = true;
    return true;
}

bool Algorithm_Filter_ScalarKalman_Update(
    Algorithm_Filter_ScalarKalman *filter,
    float measurement,
    float *output)
{
    float covariance;
    float gain;
    float value;

    if ((filter == NULL) || !filter->initialized || (output == NULL) ||
        !isfinite(measurement)) {
        return false;
    }
    if (!filter->seeded) {
        if (!Algorithm_Filter_ScalarKalman_SetState(filter, measurement)) {
            return false;
        }
        *output = measurement;
        return true;
    }
    covariance = filter->covariance + filter->process_noise;
    gain = covariance / (covariance + filter->measurement_noise);
    value = filter->value + gain * (measurement - filter->value);
    covariance *= 1.0F - gain;
    if (!isfinite(gain) || !isfinite(value) || !isfinite(covariance)) {
        return false;
    }
    filter->value = value;
    filter->covariance = covariance;
    *output = value;
    return true;
}
