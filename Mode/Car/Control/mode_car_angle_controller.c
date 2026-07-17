#include "mode_car_control.h"

#include <math.h>
#include <stddef.h>

static bool finite_value(float value)
{
    return isfinite(value) != 0;
}

static bool valid_config(
    const Mode_Car_AngleControllerConfig *config)
{
    return (config != NULL) && finite_value(config->kp) &&
        finite_value(config->ki) && finite_value(config->kd) &&
        finite_value(config->integral_speed_limit) &&
        finite_value(config->maximum_wheel_speed) &&
        finite_value(config->angle_tolerance_deg) &&
        finite_value(config->rate_tolerance_deg_per_second) &&
        (config->kp >= 0.0F) && (config->ki >= 0.0F) &&
        (config->kd >= 0.0F) &&
        (config->integral_speed_limit >= 0.0F) &&
        (config->integral_speed_limit <= config->maximum_wheel_speed) &&
        (config->maximum_wheel_speed > 0.0F) &&
        (config->angle_tolerance_deg >= 0.0F) &&
        (config->angle_tolerance_deg < 180.0F) &&
        (config->rate_tolerance_deg_per_second >= 0.0F);
}

Mode_Car_AngleControllerConfig
Mode_Car_AngleController_DefaultConfig(void)
{
    const Mode_Car_AngleControllerConfig config = {
        .kp = 3.0F,
        .ki = 0.05F,
        .kd = 1.0F,
        .integral_speed_limit = 10.0F,
        .maximum_wheel_speed = 160.0F,
        .angle_tolerance_deg = 0.5F,
        .rate_tolerance_deg_per_second = 3.0F,
    };
    return config;
}

bool Mode_Car_AngleController_Init(
    Mode_Car_AngleController *controller,
    const Mode_Car_AngleControllerConfig *config)
{
    if (controller != NULL) {
        *controller = (Mode_Car_AngleController){0};
    }
    if ((controller == NULL) || !valid_config(config)) {
        return false;
    }
    controller->config = *config;
    controller->initialized = true;
    return true;
}

void Mode_Car_AngleController_Reset(
    Mode_Car_AngleController *controller)
{
    if ((controller == NULL) || !controller->initialized) {
        return;
    }
    controller->integral_wheel_speed = 0.0F;
}

bool Mode_Car_AngleController_Update(
    Mode_Car_AngleController *controller,
    const Mode_Car_AngleControllerInput *input,
    Mode_Car_AngleControllerOutput *output)
{
    float error;
    float proportional;
    float damping;
    float integral_candidate;
    float raw_turn_speed;
    float turn_speed;
    bool within_tolerance;

    if (output != NULL) {
        *output = (Mode_Car_AngleControllerOutput){0};
    }
    if ((controller == NULL) || !controller->initialized ||
        !valid_config(&controller->config) || (input == NULL) ||
        (output == NULL) || !finite_value(input->dt_seconds) ||
        (input->dt_seconds <= 0.0F) ||
        !finite_value(input->target_yaw_deg) ||
        !finite_value(input->measured_yaw_deg) ||
        !finite_value(input->measured_yaw_rate_deg_per_second) ||
        !Algorithm_Control_AngleErrorDeg(input->target_yaw_deg,
            input->measured_yaw_deg, &error)) {
        return false;
    }

    within_tolerance =
        (fabsf(error) <= controller->config.angle_tolerance_deg) &&
        (fabsf(input->measured_yaw_rate_deg_per_second) <=
            controller->config.rate_tolerance_deg_per_second);
    if (within_tolerance) {
        controller->integral_wheel_speed = 0.0F;
        output->angle_error_deg = error;
        output->within_tolerance = true;
        return true;
    }

    proportional = controller->config.kp * error;
    damping = -controller->config.kd *
        input->measured_yaw_rate_deg_per_second;
    integral_candidate = controller->integral_wheel_speed +
        (controller->config.ki * error * input->dt_seconds);
    if (!finite_value(proportional) || !finite_value(damping) ||
        !finite_value(integral_candidate) ||
        !Algorithm_Control_Clamp(integral_candidate,
            -controller->config.integral_speed_limit,
            controller->config.integral_speed_limit,
            &integral_candidate)) {
        return false;
    }

    raw_turn_speed = proportional + integral_candidate + damping;
    if (!finite_value(raw_turn_speed)) {
        return false;
    }
    if (((raw_turn_speed > controller->config.maximum_wheel_speed) &&
            (error > 0.0F)) ||
        ((raw_turn_speed < -controller->config.maximum_wheel_speed) &&
            (error < 0.0F))) {
        integral_candidate = controller->integral_wheel_speed;
        raw_turn_speed = proportional + integral_candidate + damping;
    }
    if (!finite_value(raw_turn_speed) ||
        !Algorithm_Control_Clamp(raw_turn_speed,
            -controller->config.maximum_wheel_speed,
            controller->config.maximum_wheel_speed, &turn_speed)) {
        return false;
    }

    controller->integral_wheel_speed = integral_candidate;
    output->angle_error_deg = error;
    output->turn_wheel_speed = turn_speed;
    output->target_left_speed = -turn_speed;
    output->target_right_speed = turn_speed;
    output->within_tolerance = false;
    return true;
}
