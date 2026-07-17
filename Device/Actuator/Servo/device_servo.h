#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_servo.h"

typedef enum {
    DEVICE_SERVO_RESULT_OK = 0,
    DEVICE_SERVO_RESULT_INVALID_ARGUMENT,
    DEVICE_SERVO_RESULT_BACKEND_ERROR,
} Device_Servo_Result;

typedef struct {
    BSP_Servo_Handle backend;
    uint32_t minimum_pulse_us;
    uint32_t maximum_pulse_us;
    float maximum_angle_degrees;
} Device_Servo_Config;

typedef struct {
    Device_Servo_Config config;
    uint32_t pulse_us;
    bool enabled;
    bool initialized;
} Device_Servo;

/* Standard hobby-servo defaults are 500..2500 us and 0..180 degrees. A
 * 270-degree servo only needs maximum_angle_degrees changed before Init. */
bool Device_Servo_DefaultConfig(
    size_t output_index, Device_Servo_Config *config);
bool Device_Servo_Init(
    Device_Servo *servo, const Device_Servo_Config *config);
Device_Servo_Result Device_Servo_Enable(Device_Servo *servo, bool enabled);
Device_Servo_Result Device_Servo_SetPulseUs(
    Device_Servo *servo, uint32_t pulse_us);
Device_Servo_Result Device_Servo_SetAngle(
    Device_Servo *servo, float angle_degrees);
bool Device_Servo_GetPulseUs(
    const Device_Servo *servo, uint32_t *pulse_us);
bool Device_Servo_IsEnabled(const Device_Servo *servo);

#endif
