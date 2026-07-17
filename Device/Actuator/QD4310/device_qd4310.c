#include "device_qd4310.h"

#include <math.h>
#include <stddef.h>

#define DEVICE_QD4310_TWO_PI           6.2831853071795864769F
#define DEVICE_QD4310_RAW_POSITIVE_MAX 32767.0F

static bool valid_motor_id(uint8_t motor_id)
{
    return motor_id <= DEVICE_QD4310_MOTOR_ID_MAX;
}

static bool valid_command(Device_QD4310_Command command)
{
    return (command == DEVICE_QD4310_COMMAND_NOP) ||
           (command == DEVICE_QD4310_COMMAND_ENABLE) ||
           (command == DEVICE_QD4310_COMMAND_DISABLE) ||
           (command == DEVICE_QD4310_COMMAND_CURRENT) ||
           (command == DEVICE_QD4310_COMMAND_SPEED) ||
           (command == DEVICE_QD4310_COMMAND_ANGLE) ||
           (command == DEVICE_QD4310_COMMAND_LOW_SPEED) ||
           (command == DEVICE_QD4310_COMMAND_STEP_ANGLE);
}

static bool scale_signed(
    float value, float full_scale, int16_t *raw)
{
    float scaled;

    if ((raw == NULL) || !isfinite(value) || !isfinite(full_scale) ||
        (full_scale <= 0.0F) || (value < -full_scale) ||
        (value > full_scale)) {
        return false;
    }
    scaled = value / full_scale * DEVICE_QD4310_RAW_POSITIVE_MAX;
    *raw = (int16_t)((scaled >= 0.0F) ? scaled + 0.5F : scaled - 0.5F);
    return true;
}

static int16_t read_i16_le(const uint8_t *data)
{
    const uint16_t value = (uint16_t)data[0] |
        ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static bool send_value(
    Device_QD4310 *motor,
    Device_QD4310_Command command,
    int16_t value)
{
    BSP_CAN_Frame frame;

    return (motor != NULL) && motor->initialized &&
        Device_QD4310_EncodeCommand(
            motor->config.motor_id, command, value, &frame) &&
        (BSP_CAN_Send(motor->config.can, &frame,
            motor->config.send_timeout_ms) == BSP_CAN_RESULT_OK);
}

bool Device_QD4310_DefaultConfig(
    BSP_CAN_Handle can, uint8_t motor_id, Device_QD4310_Config *config)
{
    if ((can == NULL) || !valid_motor_id(motor_id) || (config == NULL)) {
        return false;
    }
    *config = (Device_QD4310_Config){
        .can = can,
        .motor_id = motor_id,
        .feedback_timeout_ms = 100U,
        .send_timeout_ms = 10U,
        .current_limit_a = DEVICE_QD4310_RATED_CURRENT_A,
        .speed_limit_rpm = DEVICE_QD4310_RATED_SPEED_RPM,
    };
    return true;
}

bool Device_QD4310_Init(
    Device_QD4310 *motor, const Device_QD4310_Config *config)
{
    BSP_CAN_Info can_info;

    if (motor != NULL) {
        *motor = (Device_QD4310){0};
    }
    if ((motor == NULL) || (config == NULL) || (config->can == NULL) ||
        !valid_motor_id(config->motor_id) ||
        (config->feedback_timeout_ms == 0U) ||
        (config->feedback_timeout_ms > (UINT32_MAX / 2U)) ||
        (config->send_timeout_ms == 0U) ||
        !isfinite(config->current_limit_a) ||
        (config->current_limit_a <= 0.0F) ||
        (config->current_limit_a > DEVICE_QD4310_PEAK_CURRENT_A) ||
        !isfinite(config->speed_limit_rpm) ||
        (config->speed_limit_rpm <= 0.0F) ||
        (config->speed_limit_rpm > DEVICE_QD4310_PEAK_SPEED_RPM) ||
        !BSP_CAN_GetInfo(config->can, &can_info) ||
        (can_info.nominal_bit_rate != DEVICE_QD4310_CAN_BIT_RATE)) {
        return false;
    }

    motor->config = *config;
    motor->initialized = true;
    return true;
}

bool Device_QD4310_EncodeCommand(
    uint8_t motor_id,
    Device_QD4310_Command command,
    int16_t value,
    BSP_CAN_Frame *frame)
{
    if ((frame == NULL) || !valid_motor_id(motor_id) ||
        !valid_command(command)) {
        return false;
    }

    *frame = (BSP_CAN_Frame){0};
    frame->identifier = DEVICE_QD4310_COMMAND_ID_BASE + motor_id;
    frame->data_length = 3U;
    frame->data[0] = (uint8_t)command;
    frame->data[1] = (uint8_t)((uint16_t)value & UINT16_C(0x00FF));
    frame->data[2] = (uint8_t)(((uint16_t)value >> 8U) & UINT16_C(0x00FF));
    return true;
}

bool Device_QD4310_Enable(Device_QD4310 *motor)
{
    if (!send_value(motor, DEVICE_QD4310_COMMAND_ENABLE, 0)) {
        return false;
    }
    motor->command_enabled = true;
    return true;
}

bool Device_QD4310_Disable(Device_QD4310 *motor)
{
    bool sent;

    if ((motor == NULL) || !motor->initialized) {
        return false;
    }
    motor->command_enabled = false;
    sent = send_value(motor, DEVICE_QD4310_COMMAND_DISABLE, 0);
    return sent;
}

bool Device_QD4310_RequestFeedback(Device_QD4310 *motor)
{
    return send_value(motor, DEVICE_QD4310_COMMAND_NOP, 0);
}

bool Device_QD4310_SetCurrent(Device_QD4310 *motor, float current_a)
{
    int16_t raw;

    if ((motor == NULL) || !motor->initialized ||
        !motor->command_enabled || !isfinite(current_a) ||
        (current_a < -motor->config.current_limit_a) ||
        (current_a > motor->config.current_limit_a) ||
        !scale_signed(current_a,
            DEVICE_QD4310_PROTOCOL_CURRENT_FULL_SCALE_A, &raw)) {
        return false;
    }
    return send_value(motor, DEVICE_QD4310_COMMAND_CURRENT, raw);
}

static bool set_speed(
    Device_QD4310 *motor,
    Device_QD4310_Command command,
    float speed_rpm)
{
    int16_t raw;

    if ((motor == NULL) || !motor->initialized ||
        !motor->command_enabled || !isfinite(speed_rpm) ||
        (speed_rpm < -motor->config.speed_limit_rpm) ||
        (speed_rpm > motor->config.speed_limit_rpm) ||
        !scale_signed(speed_rpm,
            DEVICE_QD4310_PROTOCOL_SPEED_FULL_SCALE_RPM, &raw)) {
        return false;
    }
    return send_value(motor, command, raw);
}

bool Device_QD4310_SetSpeed(Device_QD4310 *motor, float speed_rpm)
{
    return set_speed(motor, DEVICE_QD4310_COMMAND_SPEED, speed_rpm);
}

bool Device_QD4310_SetLowSpeed(Device_QD4310 *motor, float speed_rpm)
{
    return set_speed(motor, DEVICE_QD4310_COMMAND_LOW_SPEED, speed_rpm);
}

bool Device_QD4310_SetAngle(Device_QD4310 *motor, float angle_rad)
{
    uint16_t raw;

    if ((motor == NULL) || !motor->initialized ||
        !motor->command_enabled || !isfinite(angle_rad) ||
        (angle_rad < 0.0F) || (angle_rad > DEVICE_QD4310_TWO_PI)) {
        return false;
    }
    raw = (uint16_t)(angle_rad / DEVICE_QD4310_TWO_PI * 65535.0F + 0.5F);
    return send_value(
        motor, DEVICE_QD4310_COMMAND_ANGLE, (int16_t)raw);
}

bool Device_QD4310_StepAngle(
    Device_QD4310 *motor, float delta_angle_rad)
{
    int16_t raw;

    if ((motor == NULL) || !motor->initialized ||
        !motor->command_enabled ||
        !scale_signed(delta_angle_rad, DEVICE_QD4310_TWO_PI, &raw)) {
        return false;
    }
    return send_value(motor, DEVICE_QD4310_COMMAND_STEP_ANGLE, raw);
}

bool Device_QD4310_ConsumeFrame(
    Device_QD4310 *motor,
    const BSP_CAN_Frame *frame,
    uint32_t now_ms)
{
    Device_QD4310_Feedback next;
    int16_t raw_current;
    int16_t raw_speed;
    uint16_t raw_angle;
    float angle;
    float delta = 0.0F;

    if ((motor == NULL) || !motor->initialized || (frame == NULL) ||
        frame->extended_identifier || frame->fd_format ||
        frame->bit_rate_switch ||
        (frame->identifier !=
            DEVICE_QD4310_FEEDBACK_ID_BASE + motor->config.motor_id) ||
        (frame->data_length != 8U)) {
        return false;
    }

    raw_current = read_i16_le(&frame->data[2]);
    raw_speed = read_i16_le(&frame->data[4]);
    raw_angle = read_u16_le(&frame->data[6]);
    angle = (float)raw_angle / 65535.0F * DEVICE_QD4310_TWO_PI;
    if (motor->feedback_seen) {
        delta = angle - motor->previous_single_turn_angle_rad;
        if (delta > (DEVICE_QD4310_TWO_PI * 0.5F)) {
            delta -= DEVICE_QD4310_TWO_PI;
        }
        else if (delta < -(DEVICE_QD4310_TWO_PI * 0.5F)) {
            delta += DEVICE_QD4310_TWO_PI;
        }
    }

    next = motor->feedback;
    next.enabled = (frame->data[0] & UINT8_C(0x01)) != 0U;
    next.current_a = (float)raw_current /
        DEVICE_QD4310_RAW_POSITIVE_MAX *
        DEVICE_QD4310_PROTOCOL_CURRENT_FULL_SCALE_A;
    next.speed_rpm = (float)raw_speed /
        DEVICE_QD4310_RAW_POSITIVE_MAX *
        DEVICE_QD4310_PROTOCOL_SPEED_FULL_SCALE_RPM;
    next.single_turn_angle_rad = angle;
    next.delta_angle_rad = delta;
    next.cumulative_angle_rad = motor->feedback_seen ?
        motor->feedback.cumulative_angle_rad + delta : angle;
    next.received_at_ms = now_ms;
    ++next.sequence;
    next.online = true;

    motor->previous_single_turn_angle_rad = angle;
    motor->feedback_seen = true;
    motor->feedback = next;
    return true;
}

bool Device_QD4310_GetFeedback(
    Device_QD4310 *motor,
    uint32_t now_ms,
    Device_QD4310_Feedback *feedback)
{
    if ((motor == NULL) || !motor->initialized || (feedback == NULL)) {
        return false;
    }
    motor->feedback.online = motor->feedback_seen &&
        ((uint32_t)(now_ms - motor->feedback.received_at_ms) <=
            motor->config.feedback_timeout_ms);
    *feedback = motor->feedback;
    return true;
}

bool Device_QD4310_PollBus(
    BSP_CAN_Handle can,
    Device_QD4310 *const motors[],
    size_t motor_count,
    uint32_t now_ms,
    size_t maximum_frames,
    size_t *processed)
{
    BSP_CAN_Frame frame;
    size_t count = 0U;

    if ((can == NULL) || (motors == NULL) || (motor_count == 0U) ||
        (maximum_frames == 0U) || (processed == NULL)) {
        return false;
    }
    for (size_t index = 0U; index < motor_count; ++index) {
        if ((motors[index] == NULL) || !motors[index]->initialized ||
            (motors[index]->config.can != can)) {
            return false;
        }
    }
    while (count < maximum_frames) {
        const BSP_CAN_Result result = BSP_CAN_ReadAvailable(can, &frame);

        if (result == BSP_CAN_RESULT_NO_DATA) {
            break;
        }
        if (result != BSP_CAN_RESULT_OK) {
            *processed = count;
            return false;
        }
        for (size_t index = 0U; index < motor_count; ++index) {
            (void)Device_QD4310_ConsumeFrame(
                motors[index], &frame, now_ms);
        }
        ++count;
    }
    *processed = count;
    return true;
}
