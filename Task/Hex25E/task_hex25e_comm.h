#ifndef TASK_HEX25E_COMM_H
#define TASK_HEX25E_COMM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_HEX25E_COMMAND_NONE = 0,
    TASK_HEX25E_COMMAND_START,
    TASK_HEX25E_COMMAND_STOP,
    TASK_HEX25E_COMMAND_STATUS,
    TASK_HEX25E_COMMAND_UNKNOWN,
} Task_Hex25E_Command;

typedef struct {
    uint32_t now_ms;
    uint32_t state;
    uint32_t fault;
    uint32_t current_node;
    uint32_t next_node;
    uint32_t visited_y_mask;
    uint32_t dwell_remaining_ms;
    bool motors_enabled;
    bool user_key_pressed;
    uint32_t user_key_press_count;
    float left_target_speed;
    float right_target_speed;
    float left_measured_speed;
    float right_measured_speed;
    int64_t left_cumulative_counts;
    int64_t right_cumulative_counts;
    float left_cumulative_distance_mm;
    float right_cumulative_distance_mm;
    float route_odometry_mm;
    int32_t left_motor_command;
    int32_t right_motor_command;
    float angular_rate_dps[3];
    float euler_deg[3];
    bool imu_online;
    bool imu_has_angular_rate;
    bool imu_has_euler;
} Task_Hex25E_Telemetry;

/* Bind the Hex25E Bluetooth and VOFA roles through Device serial transports. */
bool Task_Hex25E_Comm_Init(void);

/* Drains buffered Bluetooth RX without waiting. Commands are ASCII START,
 * STOP and STATUS (case-insensitive). CR/LF terminates immediately; an
 * unterminated command is completed after a short RX-idle interval. */
bool Task_Hex25E_Comm_PollCommand(
    uint32_t now_ms, Task_Hex25E_Command *command);

/* VOFA+ FireWater: fixed-column numeric CSV with a categorized header. */
bool Task_Hex25E_Comm_SendVofa(
    const Task_Hex25E_Telemetry *telemetry);

#endif
