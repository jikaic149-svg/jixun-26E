#ifndef ALGORITHM_FILTER_H
#define ALGORITHM_FILTER_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    float time_constant_seconds;
    float value;
    bool initialized;
} Algorithm_Filter_LowPass;

bool Algorithm_Filter_LowPass_Init(
    Algorithm_Filter_LowPass *filter,
    float time_constant_seconds,
    float initial_value);
bool Algorithm_Filter_LowPass_Update(
    Algorithm_Filter_LowPass *filter,
    float input,
    float dt_seconds,
    float *output);
void Algorithm_Filter_LowPass_Reset(
    Algorithm_Filter_LowPass *filter, float value);

/* Caller supplies storage so window size and RAM use remain explicit. */
typedef struct {
    float *storage;
    size_t capacity;
    size_t count;
    size_t next;
    float sum;
    bool initialized;
} Algorithm_Filter_MovingAverage;

bool Algorithm_Filter_MovingAverage_Init(
    Algorithm_Filter_MovingAverage *filter,
    float *storage,
    size_t capacity);
bool Algorithm_Filter_MovingAverage_Update(
    Algorithm_Filter_MovingAverage *filter,
    float input,
    float *output);
void Algorithm_Filter_MovingAverage_Reset(
    Algorithm_Filter_MovingAverage *filter);

bool Algorithm_Filter_Median3(
    float a, float b, float c, float *output);

typedef enum {
    ALGORITHM_FILTER_BIQUAD_LOWPASS = 0,
    ALGORITHM_FILTER_BIQUAD_HIGHPASS,
    ALGORITHM_FILTER_BIQUAD_BANDPASS_PEAK,
    ALGORITHM_FILTER_BIQUAD_NOTCH,
} Algorithm_Filter_BiquadType;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
    bool initialized;
} Algorithm_Filter_Biquad;

bool Algorithm_Filter_Biquad_Init(
    Algorithm_Filter_Biquad *filter,
    Algorithm_Filter_BiquadType type,
    float sample_rate_hz,
    float cutoff_hz,
    float q);
bool Algorithm_Filter_Biquad_Update(
    Algorithm_Filter_Biquad *filter,
    float input,
    float *output);
void Algorithm_Filter_Biquad_Reset(Algorithm_Filter_Biquad *filter);

typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;
    float angle_deg;
    float bias_dps;
    float rate_dps;
    float p00;
    float p01;
    float p10;
    float p11;
    bool initialized;
} Algorithm_Filter_AngleKalman;

bool Algorithm_Filter_AngleKalman_Init(
    Algorithm_Filter_AngleKalman *filter,
    float q_angle,
    float q_bias,
    float r_measure,
    float initial_angle_deg);
bool Algorithm_Filter_AngleKalman_SetAngle(
    Algorithm_Filter_AngleKalman *filter, float angle_deg);
bool Algorithm_Filter_AngleKalman_Predict(
    Algorithm_Filter_AngleKalman *filter,
    float gyro_rate_dps,
    float dt_seconds,
    float *angle_deg);
bool Algorithm_Filter_AngleKalman_Correct(
    Algorithm_Filter_AngleKalman *filter,
    float acceleration_angle_deg,
    float *angle_deg);
bool Algorithm_Filter_AngleKalman_Update(
    Algorithm_Filter_AngleKalman *filter,
    float acceleration_angle_deg,
    float gyro_rate_dps,
    float dt_seconds,
    float *angle_deg);
bool Algorithm_Filter_AngleKalman_GetRate(
    const Algorithm_Filter_AngleKalman *filter, float *rate_dps);

typedef struct {
    float process_noise;
    float measurement_noise;
    float value;
    float covariance;
    bool seeded;
    bool initialized;
} Algorithm_Filter_ScalarKalman;

bool Algorithm_Filter_ScalarKalman_Init(
    Algorithm_Filter_ScalarKalman *filter,
    float process_noise,
    float measurement_noise);
bool Algorithm_Filter_ScalarKalman_SetState(
    Algorithm_Filter_ScalarKalman *filter, float value);
bool Algorithm_Filter_ScalarKalman_Update(
    Algorithm_Filter_ScalarKalman *filter,
    float measurement,
    float *output);

#endif
