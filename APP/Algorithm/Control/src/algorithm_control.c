#include "algorithm_control.h"

#include <math.h>
#include <stddef.h>

static bool finite_value(float value)
{
    return isfinite(value) != 0;
}

static bool valid_range(float minimum, float maximum)
{
    return finite_value(minimum) && finite_value(maximum) &&
        (minimum <= maximum);
}

static float clamp_valid(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool Algorithm_Control_Clamp(
    float value, float minimum, float maximum, float *output)
{
    if ((output == NULL) || !finite_value(value) ||
        !valid_range(minimum, maximum)) {
        return false;
    }
    *output = clamp_valid(value, minimum, maximum);
    return true;
}

bool Algorithm_Control_Deadband(
    float value, float half_width, float *output)
{
    if ((output == NULL) || !finite_value(value) ||
        !finite_value(half_width) || (half_width < 0.0F)) {
        return false;
    }
    *output = (fabsf(value) <= half_width) ? 0.0F : value;
    return true;
}

bool Algorithm_Control_NormalizeAngleDeg(float angle_deg, float *output_deg)
{
    float normalized;

    if ((output_deg == NULL) || !finite_value(angle_deg)) {
        return false;
    }

    normalized = fmodf(angle_deg, 360.0F);
    if (normalized >= 180.0F) {
        normalized -= 360.0F;
    }
    else if (normalized < -180.0F) {
        normalized += 360.0F;
    }
    *output_deg = normalized;
    return true;
}

bool Algorithm_Control_AngleErrorDeg(
    float target_deg, float measured_deg, float *error_deg)
{
    float difference;

    if ((error_deg == NULL) || !finite_value(target_deg) ||
        !finite_value(measured_deg)) {
        return false;
    }
    difference = target_deg - measured_deg;
    if (!finite_value(difference)) {
        return false;
    }
    return Algorithm_Control_NormalizeAngleDeg(difference, error_deg);
}

bool Algorithm_Control_Slew(
    float current,
    float target,
    float rate_per_second,
    float dt_seconds,
    float *output)
{
    float maximum_step;
    float difference;

    if ((output == NULL) || !finite_value(current) || !finite_value(target) ||
        !finite_value(rate_per_second) || !finite_value(dt_seconds) ||
        (rate_per_second < 0.0F) || (dt_seconds <= 0.0F)) {
        return false;
    }
    maximum_step = rate_per_second * dt_seconds;
    difference = target - current;
    if (!finite_value(maximum_step) || !finite_value(difference)) {
        return false;
    }
    difference = clamp_valid(difference, -maximum_step, maximum_step);
    if (!finite_value(current + difference)) {
        return false;
    }
    *output = current + difference;
    return true;
}

bool Algorithm_Control_PID_Init(
    Algorithm_Control_PID *pid,
    const Algorithm_Control_PIDConfig *config)
{
    if ((pid == NULL) || (config == NULL) ||
        !finite_value(config->kp) || !finite_value(config->ki) ||
        !finite_value(config->kd) ||
        !valid_range(config->integral_min, config->integral_max) ||
        !valid_range(config->output_min, config->output_max) ||
        !finite_value(config->integral_separation) ||
        ((config->derivative_mode != ALGORITHM_CONTROL_DERIVATIVE_ON_ERROR) &&
            (config->derivative_mode !=
                ALGORITHM_CONTROL_DERIVATIVE_ON_MEASUREMENT))) {
        return false;
    }

    pid->config = *config;
    pid->integral = 0.0F;
    pid->previous_error = 0.0F;
    pid->previous_measurement = 0.0F;
    pid->initialized = false;
    return true;
}

void Algorithm_Control_PID_Reset(Algorithm_Control_PID *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0F;
    pid->previous_error = 0.0F;
    pid->previous_measurement = 0.0F;
    pid->initialized = false;
}

bool Algorithm_Control_PID_Update(
    Algorithm_Control_PID *pid,
    float setpoint,
    float measurement,
    float dt_seconds,
    float *output)
{
    float error;
    float next_integral;
    float derivative = 0.0F;
    float next_output;

    if ((pid == NULL) || (output == NULL) || !finite_value(setpoint) ||
        !finite_value(measurement) || !finite_value(dt_seconds) ||
        (dt_seconds <= 0.0F)) {
        return false;
    }

    error = setpoint - measurement;
    if (!finite_value(error)) {
        return false;
    }

    next_integral = pid->integral;
    if ((pid->config.integral_separation <= 0.0F) ||
        (fabsf(error) <= pid->config.integral_separation)) {
        const float candidate = next_integral + (error * dt_seconds);

        if (!finite_value(candidate)) {
            return false;
        }
        next_integral = clamp_valid(candidate,
            pid->config.integral_min, pid->config.integral_max);
    }

    if (pid->initialized) {
        if (pid->config.derivative_mode ==
            ALGORITHM_CONTROL_DERIVATIVE_ON_ERROR) {
            derivative = (error - pid->previous_error) / dt_seconds;
        }
        else {
            derivative =
                -(measurement - pid->previous_measurement) / dt_seconds;
        }
        if (!finite_value(derivative)) {
            return false;
        }
    }

    next_output = (pid->config.kp * error) +
        (pid->config.ki * next_integral) +
        (pid->config.kd * derivative);
    if (!finite_value(next_output)) {
        return false;
    }
    next_output = clamp_valid(next_output,
        pid->config.output_min, pid->config.output_max);

    pid->integral = next_integral;
    pid->previous_error = error;
    pid->previous_measurement = measurement;
    pid->initialized = true;
    *output = next_output;
    return true;
}
