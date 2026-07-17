#ifndef DEVICE_MOTOR_DEVICE_MOTOR_H
#define DEVICE_MOTOR_DEVICE_MOTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_motor.h"

#define DEVICE_MOTOR_COMMAND_MIN (-10000)
#define DEVICE_MOTOR_COMMAND_MAX 10000

typedef enum {
    DEVICE_MOTOR_RESULT_OK = 0,
    DEVICE_MOTOR_RESULT_INVALID_ARGUMENT,
    DEVICE_MOTOR_RESULT_LOCKED,
    DEVICE_MOTOR_RESULT_BACKEND_ERROR,
} Device_Motor_Result;

typedef struct {
    BSP_Motor_Handle backend;
    BSP_Motor_Mode zero_mode;
    bool positive_is_forward;
} Device_Motor_Config;

typedef struct {
    Device_Motor_Config config;
    int32_t command;
    bool locked;
    bool initialized;
} Device_Motor;

/* Build the safe default binding for a logical motor: brake at zero and
 * positive command as forward. Product mechanics may adjust polarity before
 * calling Init. */
bool Device_Motor_DefaultConfig(
    size_t output_index, Device_Motor_Config *config);

/* Init immediately applies a zero command and starts locked. */
bool Device_Motor_Init(
    Device_Motor *motor, const Device_Motor_Config *config);

/* Unlocking first proves that the backend accepts a zero command, then clears
 * the software lock. Locking changes software state before requesting zero;
 * a backend error can never make later commands bypass the lock. */
Device_Motor_Result Device_Motor_SetUnlocked(
    Device_Motor *motor, bool unlocked);
bool Device_Motor_IsLocked(const Device_Motor *motor);

/* Signed logical command in [-10000,10000]. Direction pins and PWM polarity
 * stay in BSP; positive_is_forward only adapts product mechanics. */
Device_Motor_Result Device_Motor_SetCommand(
    Device_Motor *motor, int32_t command);
Device_Motor_Result Device_Motor_Stop(Device_Motor *motor);
bool Device_Motor_GetCommand(const Device_Motor *motor, int32_t *command);

#endif
