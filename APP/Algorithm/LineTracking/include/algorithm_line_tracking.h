#ifndef ALGORITHM_LINE_TRACKING_H
#define ALGORITHM_LINE_TRACKING_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    /* Prevents noise amplification while left + right is close to zero. */
    float denominator_epsilon;
    /* 0 uses only centroid; 1 uses only difference-over-sum. */
    float difference_weight;
    /* Zero disables filtering. */
    float low_pass_time_constant_seconds;
} Algorithm_LineTracking_Config;

typedef struct {
    const float *strength;
    const float *position_weight;
    size_t channel_count;
    float target_position;
    float dt_seconds;
} Algorithm_LineTracking_Input;

typedef struct {
    float left_level;
    float right_level;
    float denominator;
    float difference_ratio;
    float difference_position;
    float centroid_position;
    float raw_position;
    float filtered_position;
    float confidence;
} Algorithm_LineTracking_Output;

typedef struct {
    Algorithm_LineTracking_Config config;
    float filtered_position;
    bool filter_initialized;
    bool initialized;
} Algorithm_LineTracking;

Algorithm_LineTracking_Config Algorithm_LineTracking_DefaultConfig(void);
bool Algorithm_LineTracking_Init(
    Algorithm_LineTracking *algorithm,
    const Algorithm_LineTracking_Config *config);
void Algorithm_LineTracking_Reset(Algorithm_LineTracking *algorithm);

/* For two sensors, difference_ratio is exactly (R-L)/(R+L+epsilon).
 * For a multi-channel array, channels left and right of target_position are
 * averaged into L/R groups, which prevents unequal group sizes from biasing
 * the result. A weighted centroid is fused in to retain distance information
 * on arrays with more than two channels. */
bool Algorithm_LineTracking_Update(
    Algorithm_LineTracking *algorithm,
    const Algorithm_LineTracking_Input *input,
    Algorithm_LineTracking_Output *output);

#endif
