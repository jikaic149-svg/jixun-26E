#include "device_motor.h"

#include <stddef.h>

static bool Device_Motor_ValidZeroMode(BSP_Motor_Mode mode)
{
    return (mode == BSP_MOTOR_MODE_COAST) ||
           (mode == BSP_MOTOR_MODE_BRAKE);
}

bool Device_Motor_DefaultConfig(
    size_t output_index, Device_Motor_Config *config)
{
    Device_Motor_Config next = {
        .backend = BSP_Motor_GetHandle(output_index),
        .zero_mode = BSP_MOTOR_MODE_BRAKE,
        .positive_is_forward = true,
    };

    if ((config == NULL) || (next.backend == NULL)) {
        return false;
    }
    *config = next;
    return true;
}

static bool Device_Motor_ApplyZero(Device_Motor *motor)
{
    const BSP_Motor_State state = {
        .mode = motor->config.zero_mode,
        .duty = 0U,
    };

    if (!BSP_Motor_Apply(motor->config.backend, state)) {
        return false;
    }
    motor->command = 0;
    return true;
}

bool Device_Motor_Init(
    Device_Motor *motor, const Device_Motor_Config *config)
{
    if (motor != NULL) {
        *motor = (Device_Motor){0};
    }
    if ((motor == NULL) || (config == NULL) ||
        (config->backend == NULL) ||
        !Device_Motor_ValidZeroMode(config->zero_mode)) {
        return false;
    }

    motor->config = *config;
    motor->locked = true;
    motor->initialized = true;
    if (!Device_Motor_ApplyZero(motor)) {
        motor->initialized = false;
        return false;
    }
    return true;
}

Device_Motor_Result Device_Motor_SetUnlocked(
    Device_Motor *motor, bool unlocked)
{
    if ((motor == NULL) || !motor->initialized) {
        return DEVICE_MOTOR_RESULT_INVALID_ARGUMENT;
    }
    if (!unlocked) {
        motor->locked = true;
        return Device_Motor_ApplyZero(motor)
            ? DEVICE_MOTOR_RESULT_OK
            : DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }

    /* Stay locked until the backend confirms a safe zero state. This also
     * prevents re-unlock after a previous lock operation failed to stop. */
    if (!Device_Motor_ApplyZero(motor)) {
        motor->locked = true;
        return DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }
    motor->locked = false;
    return DEVICE_MOTOR_RESULT_OK;
}

bool Device_Motor_IsLocked(const Device_Motor *motor)
{
    return (motor == NULL) || !motor->initialized || motor->locked;
}

Device_Motor_Result Device_Motor_SetCommand(
    Device_Motor *motor, int32_t command)
{
    BSP_Motor_State state;
    uint32_t magnitude;
    bool forward;

    if ((motor == NULL) || !motor->initialized ||
        (command < DEVICE_MOTOR_COMMAND_MIN) ||
        (command > DEVICE_MOTOR_COMMAND_MAX)) {
        return DEVICE_MOTOR_RESULT_INVALID_ARGUMENT;
    }
    if (command == 0) {
        return Device_Motor_ApplyZero(motor)
            ? DEVICE_MOTOR_RESULT_OK
            : DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }
    if (motor->locked) {
        return Device_Motor_ApplyZero(motor)
            ? DEVICE_MOTOR_RESULT_LOCKED
            : DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }

    forward = command > 0;
    if (!motor->config.positive_is_forward) {
        forward = !forward;
    }
    magnitude = (command > 0)
        ? (uint32_t)command
        : (uint32_t)(-command);
    state.mode = forward
        ? BSP_MOTOR_MODE_FORWARD
        : BSP_MOTOR_MODE_REVERSE;
    state.duty = magnitude;
    if (!BSP_Motor_Apply(motor->config.backend, state)) {
        motor->locked = true;
        (void)Device_Motor_ApplyZero(motor);
        return DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }
    motor->command = command;
    return DEVICE_MOTOR_RESULT_OK;
}

Device_Motor_Result Device_Motor_Stop(Device_Motor *motor)
{
    if ((motor == NULL) || !motor->initialized) {
        return DEVICE_MOTOR_RESULT_INVALID_ARGUMENT;
    }
    if (!Device_Motor_ApplyZero(motor)) {
        motor->locked = true;
        return DEVICE_MOTOR_RESULT_BACKEND_ERROR;
    }
    return DEVICE_MOTOR_RESULT_OK;
}

bool Device_Motor_GetCommand(const Device_Motor *motor, int32_t *command)
{
    if ((motor == NULL) || !motor->initialized || (command == NULL)) {
        return false;
    }
    *command = motor->command;
    return true;
}
