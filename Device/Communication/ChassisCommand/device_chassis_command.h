#ifndef DEVICE_CHASSIS_COMMAND_H
#define DEVICE_CHASSIS_COMMAND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEVICE_CHASSIS_COMMAND_MODE_STAND = 0,
    DEVICE_CHASSIS_COMMAND_MODE_FORWARD,
    DEVICE_CHASSIS_COMMAND_MODE_BACKWARD,
    DEVICE_CHASSIS_COMMAND_MODE_TURN_LEFT,
    DEVICE_CHASSIS_COMMAND_MODE_TURN_RIGHT,
    DEVICE_CHASSIS_COMMAND_MODE_TRACKING,
    DEVICE_CHASSIS_COMMAND_MODE_SQUARE_TRACKING,
    DEVICE_CHASSIS_COMMAND_MODE_TARGET_AIM,
} Device_ChassisCommand_Mode;

typedef enum {
    DEVICE_CHASSIS_COMMAND_PARAMETER_KP = 0,
    DEVICE_CHASSIS_COMMAND_PARAMETER_KI,
    DEVICE_CHASSIS_COMMAND_PARAMETER_KD,
    DEVICE_CHASSIS_COMMAND_PARAMETER_SPEED_LIMIT_RPM,
    DEVICE_CHASSIS_COMMAND_PARAMETER_CURRENT_LIMIT_A,
    DEVICE_CHASSIS_COMMAND_PARAMETER_CURRENT_SLEW_A_PER_SEC,
} Device_ChassisCommand_RuntimeParameter;

typedef enum {
    DEVICE_CHASSIS_COMMAND_MODE = 0,
    DEVICE_CHASSIS_COMMAND_MANUAL_BIAS,
    DEVICE_CHASSIS_COMMAND_RUNTIME_PARAMETER,
    DEVICE_CHASSIS_COMMAND_WHEEL_SPEED,
    DEVICE_CHASSIS_COMMAND_COMPETITION_TASK,
    DEVICE_CHASSIS_COMMAND_CALIBRATION_LASER,
} Device_ChassisCommand_Type;

typedef struct {
    Device_ChassisCommand_Type type;
    Device_ChassisCommand_Mode mode;
    Device_ChassisCommand_RuntimeParameter parameter;
    float value;
    uint8_t task_id;
    bool enabled;
    bool vision_laser_requested;
    bool static_target_requested;
} Device_ChassisCommand_Event;

/* Parse one complete line without CR/LF. The parser is stateless and never
 * calls Mode or Task; the owning serialized task applies the typed event. */
bool Device_ChassisCommand_ParseLine(
    const uint8_t *line,
    size_t length,
    Device_ChassisCommand_Event *event);

#endif
