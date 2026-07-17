#ifndef TASK_HEX25E_H
#define TASK_HEX25E_H

#include "device_encoder.h"
#include "device_led.h"
#include "device_motor.h"
#include "device_rgb.h"
#include "device_servo.h"
#include "hex25e_route.h"
#include "mode_car.h"
#include "mode_car_control.h"

#include <stdbool.h>
#include <stdint.h>

#define TASK_HEX25E_HEADING_LOG_CAPACITY 14U

typedef enum {
    TASK_HEX25E_STATE_STOPPED = 0,
    TASK_HEX25E_STATE_EXIT_NODE,
    TASK_HEX25E_STATE_DRIVE_EDGE,
    TASK_HEX25E_STATE_CENTER_NODE,
    TASK_HEX25E_STATE_DWELL_Y,
    TASK_HEX25E_STATE_TURN,
    TASK_HEX25E_STATE_DONE,
    TASK_HEX25E_STATE_FAULT,
} Task_Hex25E_State;

typedef enum {
    TASK_HEX25E_FAULT_NONE = 0,
    TASK_HEX25E_FAULT_CONFIGURATION,
    TASK_HEX25E_FAULT_SENSOR,
    TASK_HEX25E_FAULT_ROUTE,
    TASK_HEX25E_FAULT_MOTOR,
} Task_Hex25E_Fault;

typedef struct {
    float yaw_deg;
    float yaw_rate_deg_per_second;
    bool valid;
} Task_Hex25E_Attitude;

typedef struct {
    Task_Hex25E_State state;
    Task_Hex25E_Fault fault;
    Hex25E_NodeId current_node;
    Hex25E_NodeId next_node;
    size_t route_index;
    uint32_t visited_y_mask;
    uint32_t dwell_remaining_ms;
    uint16_t heading_letter_mask;
    uint8_t heading_letter_count;
    char current_heading_letter;
    char heading_letter_log[TASK_HEX25E_HEADING_LOG_CAPACITY];
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
    bool motors_enabled;
    bool done;
} Task_Hex25E_Status;

typedef struct {
    Device_Encoder encoder[2];
    Device_Motor motor[2];
    Device_LED indicator_led[2];
    Device_RGB indicator_rgb;
    Device_Servo servo;
    Mode_Car_SpeedController speed_controller;
    Mode_Car_AngleController angle_controller;
    Mode_Car_SpinRateContext spin_rate_controller;
    Task_Hex25E_State state;
    Task_Hex25E_Fault fault;
    size_t route_index;
    uint32_t visited_y_mask;
    uint32_t state_started_ms;
    uint32_t angle_stable_started_ms;
    float distance_snapshot_mm;
    float previous_average_distance_mm;
    float route_odometry_mm;
    float angle_target_yaw_deg;
    float turn_direction;
    float turn_target_travel_deg;
    float turn_accumulated_deg;
    float turn_previous_yaw_deg;
    uint16_t heading_letter_mask;
    uint8_t heading_letter_count;
    uint8_t turn_incoming_heading;
    char current_heading_letter;
    char heading_letter_log[TASK_HEX25E_HEADING_LOG_CAPACITY];
    float left_target_speed;
    float right_target_speed;
    float left_measured_speed;
    float right_measured_speed;
    int64_t left_cumulative_counts;
    int64_t right_cumulative_counts;
    float left_cumulative_distance_mm;
    float right_cumulative_distance_mm;
    int32_t left_motor_command;
    int32_t right_motor_command;
    bool angle_stable_timing;
    bool turn_rate_active;
    bool turn_pause_after_full_rotation;
    bool turn_intermediate_pause_active;
    bool route_heading_ready;
    bool straight_distance_reset_pending;
    bool straight_motion_last_cycle;
    bool has_previous_average_distance;
    bool indicator_led_available[2];
    bool indicator_rgb_available;
    bool motors_enabled;
    bool initialized;
} Task_Hex25E_Context;

bool Task_Hex25E_Init(Task_Hex25E_Context *context, uint32_t now_ms);
bool Task_Hex25E_Start(Task_Hex25E_Context *context, uint32_t now_ms);
bool Task_Hex25E_Update(
    Task_Hex25E_Context *context,
    uint32_t now_ms,
    float dt_seconds,
    const Task_Hex25E_Attitude *attitude,
    Task_Hex25E_Status *status);
void Task_Hex25E_Stop(Task_Hex25E_Context *context);

#endif
