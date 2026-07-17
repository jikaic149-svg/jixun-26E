#include "mode_car_control.h"

#include <math.h>
#include <stddef.h>

static bool finite_value(float value)
{
    return isfinite(value) != 0;
}

static bool valid_band(const Mode_Car_SpeedBandConfig *band)
{
    return (band != NULL) && finite_value(band->kp) &&
        finite_value(band->ki) && finite_value(band->kd) &&
        (band->kp >= 0.0F) && (band->ki >= 0.0F) &&
        (band->kd >= 0.0F);
}

static bool valid_config(const Mode_Car_SpeedControllerConfig *config)
{
    return (config != NULL) && valid_band(&config->low_speed) &&
        valid_band(&config->high_speed) &&
        finite_value(config->high_speed_threshold) &&
        finite_value(config->error_deadband) &&
        finite_value(config->feedforward_percent_per_speed) &&
        finite_value(config->target_smoothing_time_constant_seconds) &&
        finite_value(config->maximum_target_speed) &&
        finite_value(config->maximum_effort_percent) &&
        finite_value(config->stopped_speed_threshold) &&
        (config->high_speed_threshold >= 0.0F) &&
        (config->error_deadband >= 0.0F) &&
        (config->target_smoothing_time_constant_seconds >= 0.0F) &&
        (config->maximum_target_speed > 0.0F) &&
        (config->maximum_effort_percent > 0.0F) &&
        (config->stopped_speed_threshold >= 0.0F) &&
        (config->high_speed_threshold <= config->maximum_target_speed) &&
        (config->feedforward_percent_per_speed >= 0.0F);
}

static const Mode_Car_SpeedBandConfig *band_for_range(
    const Mode_Car_SpeedControllerConfig *config,
    Mode_Car_SpeedRange range)
{
    return (range == MODE_CAR_SPEED_RANGE_HIGH) ?
        &config->high_speed : &config->low_speed;
}

static bool init_wheel(
    Mode_Car_WheelController *wheel,
    const Mode_Car_SpeedControllerConfig *config)
{
    if ((wheel == NULL) || (config == NULL) ||
        !valid_band(&config->low_speed)) {
        return false;
    }
    *wheel = (Mode_Car_WheelController){0};
    wheel->smoothed_target = 0.0F;
    wheel->integral_effort_percent = 0.0F;
    wheel->previous_measurement = 0.0F;
    wheel->range = MODE_CAR_SPEED_RANGE_LOW;
    wheel->measurement_initialized = false;
    return true;
}

static void stop_output(Mode_Car_SpeedControllerOutput *output)
{
    if (output == NULL) {
        return;
    }
    output->smoothed_left_target = 0.0F;
    output->smoothed_right_target = 0.0F;
    output->left_effort_percent = 0.0F;
    output->right_effort_percent = 0.0F;
    output->left_range = MODE_CAR_SPEED_RANGE_LOW;
    output->right_range = MODE_CAR_SPEED_RANGE_LOW;
}

static bool update_wheel(
    Mode_Car_WheelController *wheel,
    const Mode_Car_SpeedControllerConfig *config,
    float requested_target,
    float measured_speed,
    float dt_seconds,
    float *effort_percent)
{
    const Mode_Car_SpeedBandConfig *band;
    Mode_Car_SpeedRange next_range;
    float target;
    float smoothing_alpha;
    float error;
    float feedforward;
    float proportional;
    float derivative = 0.0F;
    float integral_candidate;
    float raw_effort;
    float effort;

    if ((wheel == NULL) || (config == NULL) || (effort_percent == NULL) ||
        !finite_value(requested_target) || !finite_value(measured_speed) ||
        !finite_value(dt_seconds) || (dt_seconds <= 0.0F) ||
        !Algorithm_Control_Clamp(requested_target,
            -config->maximum_target_speed,
            config->maximum_target_speed, &target)) {
        return false;
    }

    if ((target != 0.0F) && (wheel->smoothed_target != 0.0F) &&
        ((target > 0.0F) != (wheel->smoothed_target > 0.0F))) {
        /* Do not carry drive integral through a commanded direction change. */
        wheel->smoothed_target = 0.0F;
        wheel->integral_effort_percent = 0.0F;
    }
    if (target == 0.0F) {
        wheel->smoothed_target = 0.0F;
    }
    else if (config->target_smoothing_time_constant_seconds == 0.0F) {
        wheel->smoothed_target = target;
    }
    else {
        smoothing_alpha = 1.0F - expf(
            -dt_seconds /
            config->target_smoothing_time_constant_seconds);
        if (!finite_value(smoothing_alpha) ||
            !Algorithm_Control_Clamp(smoothing_alpha, 0.0F, 1.0F,
                &smoothing_alpha)) {
            return false;
        }
        wheel->smoothed_target +=
            (target - wheel->smoothed_target) * smoothing_alpha;
        if (!finite_value(wheel->smoothed_target)) {
            return false;
        }
    }

    next_range = (fabsf(wheel->smoothed_target) >
        config->high_speed_threshold) ? MODE_CAR_SPEED_RANGE_HIGH :
        MODE_CAR_SPEED_RANGE_LOW;
    wheel->range = next_range;
    band = band_for_range(config, next_range);

    error = wheel->smoothed_target - measured_speed;
    if (!finite_value(error)) {
        return false;
    }
    if (fabsf(error) <= config->error_deadband) {
        error = 0.0F;
    }

    feedforward = wheel->smoothed_target *
        config->feedforward_percent_per_speed;
    proportional = band->kp * error;
    if (wheel->measurement_initialized) {
        derivative = -band->kd *
            ((measured_speed - wheel->previous_measurement) / dt_seconds);
    }
    if (!finite_value(feedforward) || !finite_value(proportional) ||
        !finite_value(derivative)) {
        return false;
    }

    /* Integrate directly in final motor-effort units.  Candidate integration
     * is rejected only when it would push an already saturated final command
     * farther into saturation.  Integration in the recovery direction stays
     * active, so feedforward error can always be removed. */
    integral_candidate = (target == 0.0F) ? 0.0F :
        wheel->integral_effort_percent +
            (band->ki * error * dt_seconds);
    if (!finite_value(integral_candidate) ||
        !Algorithm_Control_Clamp(integral_candidate,
            -config->maximum_effort_percent,
            config->maximum_effort_percent, &integral_candidate)) {
        return false;
    }
    raw_effort = feedforward + proportional + integral_candidate +
        derivative;
    if (!finite_value(raw_effort)) {
        return false;
    }
    if ((target != 0.0F) &&
        (((raw_effort > config->maximum_effort_percent) &&
            (error > 0.0F)) ||
        ((raw_effort < -config->maximum_effort_percent) &&
            (error < 0.0F)))) {
        integral_candidate = wheel->integral_effort_percent;
        raw_effort = feedforward + proportional + integral_candidate +
            derivative;
    }
    if (!finite_value(raw_effort) ||
        !Algorithm_Control_Clamp(raw_effort,
            -config->maximum_effort_percent,
            config->maximum_effort_percent, &effort)) {
        return false;
    }

    if ((target == 0.0F) &&
        (fabsf(measured_speed) < config->stopped_speed_threshold)) {
        effort = 0.0F;
    }

    wheel->integral_effort_percent = integral_candidate;
    wheel->previous_measurement = measured_speed;
    wheel->measurement_initialized = true;
    *effort_percent = effort;
    return true;
}

Mode_Car_SpeedControllerConfig
Mode_Car_SpeedController_DefaultConfig(void)
{
    const Mode_Car_SpeedControllerConfig config = {
        .low_speed = {
            .kp = 0.15F,
            .ki = 0.30F,
            .kd = 0.0005F,
        },
        .high_speed = {
            .kp = 0.12F,
            .ki = 0.20F,
            .kd = 0.0005F,
        },
        .high_speed_threshold = 300.0F,
        .error_deadband = 2.0F,
        /* Initial plant estimate from the measured steady wheel data:
         * 260 mm/s starts near 14.3 percent instead of the old 39 percent. */
        .feedforward_percent_per_speed = 0.055F,
        /* Equivalent to alpha=0.15 at the legacy 5 ms update period. */
        .target_smoothing_time_constant_seconds = 0.030765F,
        .maximum_target_speed = 600.0F,
        .maximum_effort_percent = 55.0F,
        .stopped_speed_threshold = 20.0F,
    };
    return config;
}

bool Mode_Car_SpeedController_Init(
    Mode_Car_SpeedController *controller,
    const Mode_Car_SpeedControllerConfig *config)
{
    Mode_Car_SpeedController next;

    if (controller != NULL) {
        controller->initialized = false;
    }
    if ((controller == NULL) || !valid_config(config)) {
        return false;
    }

    next.config = *config;
    next.initialized = false;
    if (!init_wheel(&next.left, config) ||
        !init_wheel(&next.right, config)) {
        return false;
    }
    next.initialized = true;
    *controller = next;
    return true;
}

void Mode_Car_SpeedController_Reset(
    Mode_Car_SpeedController *controller)
{
    if ((controller == NULL) || !controller->initialized) {
        return;
    }
    (void)init_wheel(&controller->left, &controller->config);
    (void)init_wheel(&controller->right, &controller->config);
}

bool Mode_Car_SpeedController_Update(
    Mode_Car_SpeedController *controller,
    const Mode_Car_SpeedControllerInput *input,
    Mode_Car_SpeedControllerOutput *output)
{
    Mode_Car_SpeedController next;
    Mode_Car_SpeedControllerOutput next_output;

    stop_output(output);
    if ((controller == NULL) || !controller->initialized ||
        (input == NULL) || (output == NULL) ||
        !finite_value(input->dt_seconds) ||
        (input->dt_seconds <= 0.0F)) {
        return false;
    }

    next = *controller;
    stop_output(&next_output);
    if (!update_wheel(&next.left, &next.config,
            input->target_left_speed, input->measured_left_speed,
            input->dt_seconds, &next_output.left_effort_percent) ||
        !update_wheel(&next.right, &next.config,
            input->target_right_speed, input->measured_right_speed,
            input->dt_seconds, &next_output.right_effort_percent)) {
        return false;
    }

    next_output.smoothed_left_target = next.left.smoothed_target;
    next_output.smoothed_right_target = next.right.smoothed_target;
    next_output.left_range = next.left.range;
    next_output.right_range = next.right.range;
    *controller = next;
    *output = next_output;
    return true;
}
