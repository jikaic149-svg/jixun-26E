#ifndef DEVICE_QD4310_H
#define DEVICE_QD4310_H

#include "bsp_can.h"

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_QD4310_COMMAND_ID_BASE  UINT32_C(0x400)
#define DEVICE_QD4310_FEEDBACK_ID_BASE UINT32_C(0x500)
#define DEVICE_QD4310_MOTOR_ID_MAX     UINT8_C(0x0F)
#define DEVICE_QD4310_CAN_BIT_RATE      UINT32_C(1000000)

/* V2.3 manual physical ratings. The binary protocol still uses wider full
 * scales, so protocol scaling and safe command limits must stay separate. */
#define DEVICE_QD4310_RATED_CURRENT_A              1.0F
#define DEVICE_QD4310_PEAK_CURRENT_A               2.0F
#define DEVICE_QD4310_RATED_SPEED_RPM              500.0F
#define DEVICE_QD4310_PEAK_SPEED_RPM               800.0F
#define DEVICE_QD4310_TORQUE_CONSTANT_NM_PER_A     0.27F
#define DEVICE_QD4310_PROTOCOL_CURRENT_FULL_SCALE_A 10.0F
#define DEVICE_QD4310_PROTOCOL_SPEED_FULL_SCALE_RPM 1000.0F

typedef enum {
    DEVICE_QD4310_COMMAND_NOP = 0x00,
    DEVICE_QD4310_COMMAND_ENABLE = 0x01,
    DEVICE_QD4310_COMMAND_DISABLE = 0x02,
    DEVICE_QD4310_COMMAND_CURRENT = 0x03,
    DEVICE_QD4310_COMMAND_SPEED = 0x04,
    DEVICE_QD4310_COMMAND_ANGLE = 0x05,
    DEVICE_QD4310_COMMAND_LOW_SPEED = 0x06,
    DEVICE_QD4310_COMMAND_STEP_ANGLE = 0x07,
} Device_QD4310_Command;

typedef struct {
    BSP_CAN_Handle can;
    uint8_t motor_id;
    uint32_t feedback_timeout_ms;
    uint32_t send_timeout_ms;
    /* Local command guards. Defaults use continuous ratings. Values may be
     * raised only as far as the V2.3 physical peak ratings. */
    float current_limit_a;
    float speed_limit_rpm;
} Device_QD4310_Config;

typedef struct {
    bool enabled;
    float current_a;
    float speed_rpm;
    float single_turn_angle_rad;
    float delta_angle_rad;
    float cumulative_angle_rad;
    uint32_t received_at_ms;
    uint32_t sequence;
    bool online;
} Device_QD4310_Feedback;

typedef struct {
    Device_QD4310_Config config;
    Device_QD4310_Feedback feedback;
    float previous_single_turn_angle_rad;
    bool feedback_seen;
    bool command_enabled;
    bool initialized;
} Device_QD4310;

bool Device_QD4310_DefaultConfig(
    BSP_CAN_Handle can, uint8_t motor_id, Device_QD4310_Config *config);
bool Device_QD4310_Init(
    Device_QD4310 *motor, const Device_QD4310_Config *config);

/* Public encoder is useful for protocol inspection and host-side tools. */
bool Device_QD4310_EncodeCommand(
    uint8_t motor_id,
    Device_QD4310_Command command,
    int16_t value,
    BSP_CAN_Frame *frame);

bool Device_QD4310_Enable(Device_QD4310 *motor);
bool Device_QD4310_Disable(Device_QD4310 *motor);
/* NOP requests one feedback frame without changing the control mode. */
bool Device_QD4310_RequestFeedback(Device_QD4310 *motor);
bool Device_QD4310_SetCurrent(Device_QD4310 *motor, float current_a);
bool Device_QD4310_SetSpeed(Device_QD4310 *motor, float speed_rpm);
bool Device_QD4310_SetLowSpeed(Device_QD4310 *motor, float speed_rpm);
bool Device_QD4310_SetAngle(Device_QD4310 *motor, float angle_rad);
bool Device_QD4310_StepAngle(Device_QD4310 *motor, float delta_angle_rad);

/* Feed every received CAN frame to each motor instance. Non-matching frames
 * return false without modifying state, so one APP dispatcher can serve many
 * QD4310 motors on the same CAN bus. */
bool Device_QD4310_ConsumeFrame(
    Device_QD4310 *motor,
    const BSP_CAN_Frame *frame,
    uint32_t now_ms);
bool Device_QD4310_GetFeedback(
    Device_QD4310 *motor,
    uint32_t now_ms,
    Device_QD4310_Feedback *feedback);

/* Poll one CAN owner and dispatch each frame to all listed QD4310 instances.
 * NO_DATA is a successful empty poll. Any bus error or invalid motor list
 * returns false; processed receives the number of CAN frames consumed. */
bool Device_QD4310_PollBus(
    BSP_CAN_Handle can,
    Device_QD4310 *const motors[],
    size_t motor_count,
    uint32_t now_ms,
    size_t maximum_frames,
    size_t *processed);

#endif
