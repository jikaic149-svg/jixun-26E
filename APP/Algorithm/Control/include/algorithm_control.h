#ifndef ALGORITHM_CONTROL_H
#define ALGORITHM_CONTROL_H

#include <stdbool.h>

typedef enum {
    ALGORITHM_CONTROL_DERIVATIVE_ON_ERROR = 0,
    ALGORITHM_CONTROL_DERIVATIVE_ON_MEASUREMENT,
} Algorithm_Control_DerivativeMode;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
    /* <= 0 disables integral separation. Otherwise integration occurs only
     * while abs(error) <= integral_separation. */
    float integral_separation;
    Algorithm_Control_DerivativeMode derivative_mode;
} Algorithm_Control_PIDConfig;

typedef struct {
    Algorithm_Control_PIDConfig config;
    float integral;
    float previous_error;
    float previous_measurement;
    bool initialized;
} Algorithm_Control_PID;

/* All functions reject non-finite values and invalid ranges. On failure an
 * output argument and PID state remain unchanged. */
bool Algorithm_Control_Clamp(
    float value, float minimum, float maximum, float *output);
bool Algorithm_Control_Deadband(
    float value, float half_width, float *output);
bool Algorithm_Control_NormalizeAngleDeg(float angle_deg, float *output_deg);
bool Algorithm_Control_AngleErrorDeg(
    float target_deg, float measured_deg, float *error_deg);

/* Move current toward target by at most rate_per_second * dt_seconds. */
bool Algorithm_Control_Slew(
    float current,
    float target,
    float rate_per_second,
    float dt_seconds,
    float *output);

bool Algorithm_Control_PID_Init(
    Algorithm_Control_PID *pid,
    const Algorithm_Control_PIDConfig *config);
/* Reset/Update require a PID instance previously initialized successfully. */
void Algorithm_Control_PID_Reset(Algorithm_Control_PID *pid);
bool Algorithm_Control_PID_Update(
    Algorithm_Control_PID *pid,
    float setpoint,
    float measurement,
    float dt_seconds,
    float *output);

#endif
