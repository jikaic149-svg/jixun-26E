#include "device_servo.h"

#include <stddef.h>

#define DEVICE_SERVO_DEFAULT_MINIMUM_PULSE_US UINT32_C(500)
#define DEVICE_SERVO_DEFAULT_MAXIMUM_PULSE_US UINT32_C(2500)
#define DEVICE_SERVO_DEFAULT_MAXIMUM_ANGLE_DEG 180.0F

bool Device_Servo_DefaultConfig(
    size_t output_index, Device_Servo_Config *config)
{
    Device_Servo_Config next = {
        .backend = BSP_Servo_GetHandle(output_index),
        .minimum_pulse_us = DEVICE_SERVO_DEFAULT_MINIMUM_PULSE_US,
        .maximum_pulse_us = DEVICE_SERVO_DEFAULT_MAXIMUM_PULSE_US,
        .maximum_angle_degrees = DEVICE_SERVO_DEFAULT_MAXIMUM_ANGLE_DEG,
    };

    if ((config == NULL) || (next.backend == NULL)) {
        return false;
    }
    *config = next;
    return true;
}

static bool config_is_valid(const Device_Servo_Config *config)
{
    uint32_t frame_period_us;

    return (config != NULL) && (config->backend != NULL) &&
        (config->minimum_pulse_us > 0U) &&
        (config->minimum_pulse_us < config->maximum_pulse_us) &&
        (config->maximum_angle_degrees > 0.0F) &&
        BSP_Servo_GetFramePeriodUs(config->backend, &frame_period_us) &&
        (config->maximum_pulse_us < frame_period_us);
}

bool Device_Servo_Init(
    Device_Servo *servo, const Device_Servo_Config *config)
{
    uint32_t center_pulse_us;

    if (servo != NULL) {
        *servo = (Device_Servo){0};
    }
    if ((servo == NULL) || !config_is_valid(config)) {
        return false;
    }

    center_pulse_us = config->minimum_pulse_us +
        ((config->maximum_pulse_us - config->minimum_pulse_us) / 2U);
    if (!BSP_Servo_SetEnabled(config->backend, false) ||
        !BSP_Servo_SetPulseUs(config->backend, center_pulse_us)) {
        return false;
    }
    servo->config = *config;
    servo->pulse_us = center_pulse_us;
    servo->initialized = true;
    return true;
}

Device_Servo_Result Device_Servo_Enable(Device_Servo *servo, bool enabled)
{
    if ((servo == NULL) || !servo->initialized) {
        return DEVICE_SERVO_RESULT_INVALID_ARGUMENT;
    }
    if (!BSP_Servo_SetEnabled(servo->config.backend, enabled)) {
        servo->enabled = false;
        return DEVICE_SERVO_RESULT_BACKEND_ERROR;
    }
    servo->enabled = enabled;
    return DEVICE_SERVO_RESULT_OK;
}

Device_Servo_Result Device_Servo_SetPulseUs(
    Device_Servo *servo, uint32_t pulse_us)
{
    if ((servo == NULL) || !servo->initialized ||
        (pulse_us < servo->config.minimum_pulse_us) ||
        (pulse_us > servo->config.maximum_pulse_us)) {
        return DEVICE_SERVO_RESULT_INVALID_ARGUMENT;
    }
    if (!BSP_Servo_SetPulseUs(servo->config.backend, pulse_us)) {
        return DEVICE_SERVO_RESULT_BACKEND_ERROR;
    }
    servo->pulse_us = pulse_us;
    return DEVICE_SERVO_RESULT_OK;
}

Device_Servo_Result Device_Servo_SetAngle(
    Device_Servo *servo, float angle_degrees)
{
    float pulse_us;

    if ((servo == NULL) || !servo->initialized ||
        !(angle_degrees >= 0.0F) ||
        (angle_degrees > servo->config.maximum_angle_degrees)) {
        return DEVICE_SERVO_RESULT_INVALID_ARGUMENT;
    }
    pulse_us = (float)servo->config.minimum_pulse_us +
        ((float)(servo->config.maximum_pulse_us -
            servo->config.minimum_pulse_us) * angle_degrees /
            servo->config.maximum_angle_degrees);
    return Device_Servo_SetPulseUs(servo, (uint32_t)(pulse_us + 0.5F));
}

bool Device_Servo_GetPulseUs(
    const Device_Servo *servo, uint32_t *pulse_us)
{
    if ((servo == NULL) || !servo->initialized || (pulse_us == NULL)) {
        return false;
    }
    *pulse_us = servo->pulse_us;
    return true;
}

bool Device_Servo_IsEnabled(const Device_Servo *servo)
{
    return (servo != NULL) && servo->initialized && servo->enabled;
}
