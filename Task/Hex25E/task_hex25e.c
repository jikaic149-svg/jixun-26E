#include "task_hex25e.h"

#include "hex25e_config.h"

#include <math.h>
#include <stddef.h>

static const uint32_t g_all_y_mask =
    (UINT32_C(1) << HEX25E_NODE_A) |
    (UINT32_C(1) << HEX25E_NODE_F) |
    (UINT32_C(1) << HEX25E_NODE_M) |
    (UINT32_C(1) << HEX25E_NODE_C) |
    (UINT32_C(1) << HEX25E_NODE_H);

static bool get_encoder_samples(
    Task_Hex25E_Context *context,
    float dt_seconds,
    Device_Encoder_Sample sample[2])
{
    return Device_Encoder_Update(&context->encoder[0], dt_seconds) &&
        Device_Encoder_Update(&context->encoder[1], dt_seconds) &&
        Device_Encoder_GetSample(&context->encoder[0], &sample[0]) &&
        Device_Encoder_GetSample(&context->encoder[1], &sample[1]);
}

static float average_distance(const Device_Encoder_Sample sample[2])
{
    return (sample[0].cumulative_distance +
        sample[1].cumulative_distance) * 0.5F;
}

static float clamp_value(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float wrap_yaw(float yaw_deg)
{
    while (yaw_deg > 180.0F) {
        yaw_deg -= 360.0F;
    }
    while (yaw_deg < -180.0F) {
        yaw_deg += 360.0F;
    }
    return yaw_deg;
}

static bool valid_attitude(const Task_Hex25E_Attitude *attitude)
{
    return (attitude != NULL) && attitude->valid &&
        isfinite(attitude->yaw_deg) &&
        isfinite(attitude->yaw_rate_deg_per_second);
}

static bool make_straight_targets(
    const Task_Hex25E_Context *context,
    const Task_Hex25E_Attitude *attitude,
    float *left_target,
    float *right_target)
{
    float target_yaw;
    float yaw_error;
    float turn_speed;

    if ((context == NULL) || !valid_attitude(attitude) ||
        (left_target == NULL) || (right_target == NULL)) {
        return false;
    }
    target_yaw = context->angle_target_yaw_deg;
    yaw_error = wrap_yaw(target_yaw - attitude->yaw_deg);
    turn_speed = HEX25E_CONFIG_STRAIGHT_HEADING_KP * yaw_error -
        HEX25E_CONFIG_STRAIGHT_HEADING_KD *
            attitude->yaw_rate_deg_per_second;
    turn_speed = clamp_value(turn_speed,
        -HEX25E_CONFIG_STRAIGHT_TURN_LIMIT_MM_S,
        HEX25E_CONFIG_STRAIGHT_TURN_LIMIT_MM_S);
    *left_target = HEX25E_CONFIG_STRAIGHT_SPEED_MM_S - turn_speed;
    *right_target = HEX25E_CONFIG_STRAIGHT_SPEED_MM_S + turn_speed;
    return true;
}

static bool stop_motors(Task_Hex25E_Context *context)
{
    Mode_Car_SpeedController_Reset(&context->speed_controller);
    context->left_target_speed = 0.0F;
    context->right_target_speed = 0.0F;
    context->left_motor_command = 0;
    context->right_motor_command = 0;
    return (Device_Motor_Stop(&context->motor[0]) ==
            DEVICE_MOTOR_RESULT_OK) &&
        (Device_Motor_Stop(&context->motor[1]) == DEVICE_MOTOR_RESULT_OK);
}

static Device_RGB_Color indicator_color(Task_Hex25E_State state)
{
    const uint8_t level = HEX25E_CONFIG_RGB_LEVEL;

    if (state == TASK_HEX25E_STATE_FAULT) {
        return (Device_RGB_Color){level, 0U, 0U};
    }
    return (Device_RGB_Color){0U, level, 0U};
}

static void update_indicators(Task_Hex25E_Context *context)
{
    bool run_on;
    bool status_on;

    if (context == NULL) {
        return;
    }
    run_on = context->state != TASK_HEX25E_STATE_FAULT;
    status_on = context->state == TASK_HEX25E_STATE_FAULT;

    if (context->indicator_led_available[0] &&
        (Device_LED_Set(&context->indicator_led[0], run_on) !=
            DEVICE_LED_RESULT_OK)) {
        context->indicator_led_available[0] = false;
    }
    if (context->indicator_led_available[1] &&
        (Device_LED_Set(&context->indicator_led[1], status_on) !=
            DEVICE_LED_RESULT_OK)) {
        context->indicator_led_available[1] = false;
    }
    if (context->indicator_rgb_available &&
        (Device_RGB_SetColor(&context->indicator_rgb,
            indicator_color(context->state)) != DEVICE_RGB_RESULT_OK)) {
        context->indicator_rgb_available = false;
    }
}

static bool apply_wheel_targets(
    Task_Hex25E_Context *context,
    float left_target,
    float right_target,
    const Device_Encoder_Sample encoder[2],
    float dt_seconds)
{
    Mode_Car_SpeedControllerInput input = {
        .dt_seconds = dt_seconds,
        .target_left_speed = left_target,
        .target_right_speed = right_target,
        .measured_left_speed = encoder[0].speed_distance_per_second,
        .measured_right_speed = encoder[1].speed_distance_per_second,
    };
    Mode_Car_SpeedControllerOutput output;
    int32_t left_command;
    int32_t right_command;

    if (!Mode_Car_SpeedController_Update(
            &context->speed_controller, &input, &output)) {
        return false;
    }
    left_command = (int32_t)lroundf(output.left_effort_percent * 100.0F);
    right_command = (int32_t)lroundf(output.right_effort_percent * 100.0F);
    context->left_motor_command = left_command;
    context->right_motor_command = right_command;
    return (Device_Motor_SetCommand(&context->motor[0], left_command) ==
            DEVICE_MOTOR_RESULT_OK) &&
        (Device_Motor_SetCommand(&context->motor[1], right_command) ==
            DEVICE_MOTOR_RESULT_OK);
}

static void enter_fault(
    Task_Hex25E_Context *context, Task_Hex25E_Fault fault)
{
    context->fault = fault;
    context->state = TASK_HEX25E_STATE_FAULT;
    context->motors_enabled = false;
    context->straight_motion_last_cycle = false;
    (void)stop_motors(context);
    (void)Device_Motor_SetUnlocked(&context->motor[0], false);
    (void)Device_Motor_SetUnlocked(&context->motor[1], false);
    (void)Device_Servo_Enable(&context->servo, false);
    update_indicators(context);
}

static char heading_letter_for(Hex25E_NodeId node, uint8_t heading)
{
    switch (node) {
    case HEX25E_NODE_A:
        if (heading == 2U) return 'M';
        if (heading == 4U) return 'N';
        if (heading == 0U) return 'O';
        break;
    case HEX25E_NODE_F:
        if (heading == 3U) return 'J';
        if (heading == 1U) return 'K';
        if (heading == 5U) return 'L';
        break;
    case HEX25E_NODE_H:
        if (heading == 0U) return 'A';
        if (heading == 4U) return 'E';
        if (heading == 2U) return 'I';
        break;
    case HEX25E_NODE_M:
        if (heading == 5U) return 'F';
        if (heading == 3U) return 'G';
        if (heading == 1U) return 'H';
        break;
    case HEX25E_NODE_C:
        if (heading == 1U) return 'B';
        if (heading == 5U) return 'C';
        if (heading == 3U) return 'D';
        break;
    default:
        break;
    }
    return '\0';
}

static void record_current_heading_letter(Task_Hex25E_Context *context)
{
    float heading_angle_deg;
    float nearest_angle_deg;
    float error_deg;
    uint8_t heading;
    char letter;
    uint16_t bit;
    const Hex25E_NodeId *route;
    size_t route_count;

    if (context == NULL) {
        return;
    }
    heading_angle_deg =
        (float)context->turn_incoming_heading * 60.0F +
        context->turn_direction * context->turn_accumulated_deg;
    while (heading_angle_deg >= 360.0F) heading_angle_deg -= 360.0F;
    while (heading_angle_deg < 0.0F) heading_angle_deg += 360.0F;
    heading = (uint8_t)(((unsigned int)
        floorf((heading_angle_deg + 30.0F) / 60.0F)) % 6U);
    nearest_angle_deg = (float)heading * 60.0F;
    error_deg = wrap_yaw(heading_angle_deg - nearest_angle_deg);
    if (fabsf(error_deg) >
        HEX25E_CONFIG_HEADING_LETTER_TOLERANCE_DEG) {
        context->current_heading_letter = '\0';
        return;
    }

    route = Hex25E_Map_GetShortestRoute(&route_count);
    if ((route == NULL) || (context->route_index >= route_count)) {
        context->current_heading_letter = '\0';
        return;
    }
    letter = heading_letter_for(route[context->route_index], heading);
    /* O is the launch/return direction and is deliberately not recorded. */
    if ((letter < 'A') || (letter > 'N')) {
        context->current_heading_letter = '\0';
        return;
    }
    context->current_heading_letter = letter;
    bit = (uint16_t)(UINT16_C(1) << (unsigned int)(letter - 'A'));
    if ((context->heading_letter_mask & bit) != 0U) {
        return;
    }
    context->heading_letter_mask |= bit;
    if (context->heading_letter_count <
        TASK_HEX25E_HEADING_LOG_CAPACITY) {
        context->heading_letter_log[context->heading_letter_count++] =
            letter;
    }
}

static bool start_turn(
    Task_Hex25E_Context *context,
    uint32_t now_ms,
    bool use_long_direction,
    float current_yaw_deg)
{
    uint8_t incoming;
    uint8_t outgoing;
    uint8_t change;

    if ((context->route_index == 0U) ||
        (context->route_index >= HEX25E_CONFIG_ROUTE_EDGE_COUNT) ||
        !isfinite(current_yaw_deg) ||
        !Hex25E_Map_GetRouteHeading(context->route_index - 1U, &incoming) ||
        !Hex25E_Map_GetRouteHeading(context->route_index, &outgoing)) {
        return false;
    }
    change = (uint8_t)((outgoing + 6U - incoming) % 6U);
    context->turn_incoming_heading = incoming;
    context->turn_rate_active = use_long_direction;
    context->turn_direction = 0.0F;
    context->turn_target_travel_deg = 0.0F;
    context->turn_accumulated_deg = 0.0F;
    context->turn_previous_yaw_deg = current_yaw_deg;
    context->turn_pause_after_full_rotation = false;
    context->turn_intermediate_pause_active = false;
    if (change == 1U) {
        context->angle_target_yaw_deg = wrap_yaw(
            context->angle_target_yaw_deg +
                HEX25E_CONFIG_ANGLE_STEP_DEG);
        if (use_long_direction) {
            context->turn_direction = -1.0F;
            context->turn_target_travel_deg =
                HEX25E_CONFIG_Y_LONG_TURN_DEG;
        }
    }
    else if (change == 5U) {
        context->angle_target_yaw_deg = wrap_yaw(
            context->angle_target_yaw_deg -
                HEX25E_CONFIG_ANGLE_STEP_DEG);
        if (use_long_direction) {
            context->turn_direction = 1.0F;
            context->turn_target_travel_deg =
                HEX25E_CONFIG_Y_LONG_TURN_DEG;
        }
    }
    else if (change == 3U) {
        context->angle_target_yaw_deg = wrap_yaw(
            context->angle_target_yaw_deg +
                HEX25E_CONFIG_U_TURN_ANGLE_DEG);
        if (use_long_direction) {
            context->turn_direction = 1.0F;
            context->turn_target_travel_deg =
                HEX25E_CONFIG_Y_U_TURN_TOTAL_DEG;
            context->turn_pause_after_full_rotation = true;
        }
    }
    else {
        return false;
    }
    if (use_long_direction) {
        record_current_heading_letter(context);
    }
    context->angle_stable_timing = false;
    Mode_Car_AngleController_Reset(&context->angle_controller);
    Mode_Car_SpinRate_Reset(&context->spin_rate_controller);
    context->state_started_ms = now_ms;
    context->state = TASK_HEX25E_STATE_TURN;
    return true;
}

static void make_status(
    const Task_Hex25E_Context *context,
    uint32_t now_ms,
    Task_Hex25E_Status *status)
{
    const Hex25E_NodeId *route;
    size_t count;

    *status = (Task_Hex25E_Status){0};
    route = Hex25E_Map_GetShortestRoute(&count);
    status->state = context->state;
    status->fault = context->fault;
    status->route_index = context->route_index;
    status->visited_y_mask = context->visited_y_mask;
    status->heading_letter_mask = context->heading_letter_mask;
    status->heading_letter_count = context->heading_letter_count;
    status->current_heading_letter = context->current_heading_letter;
    for (size_t index = 0U;
         index < TASK_HEX25E_HEADING_LOG_CAPACITY; ++index) {
        status->heading_letter_log[index] =
            context->heading_letter_log[index];
    }
    status->left_target_speed = context->left_target_speed;
    status->right_target_speed = context->right_target_speed;
    status->left_measured_speed = context->left_measured_speed;
    status->right_measured_speed = context->right_measured_speed;
    status->left_cumulative_counts = context->left_cumulative_counts;
    status->right_cumulative_counts = context->right_cumulative_counts;
    status->left_cumulative_distance_mm =
        context->left_cumulative_distance_mm;
    status->right_cumulative_distance_mm =
        context->right_cumulative_distance_mm;
    status->route_odometry_mm = context->route_odometry_mm;
    status->left_motor_command = context->left_motor_command;
    status->right_motor_command = context->right_motor_command;
    status->motors_enabled = context->motors_enabled;
    status->done = context->state == TASK_HEX25E_STATE_DONE;
    if ((route != NULL) && (context->route_index < count)) {
        status->current_node = route[context->route_index];
        status->next_node = (context->route_index + 1U < count) ?
            route[context->route_index + 1U] : route[context->route_index];
    }
    if (context->state == TASK_HEX25E_STATE_DWELL_Y) {
        uint32_t elapsed = (uint32_t)(now_ms - context->state_started_ms);
        status->dwell_remaining_ms = (elapsed < HEX25E_CONFIG_Y_DWELL_MS) ?
            HEX25E_CONFIG_Y_DWELL_MS - elapsed : 0U;
    }
}

bool Task_Hex25E_Init(Task_Hex25E_Context *context, uint32_t now_ms)
{
    Device_Encoder_Config encoder_config[2];
    Device_Motor_Config motor_config[2];
    Mode_Car_SpeedControllerConfig speed_config;
    Mode_Car_AngleControllerConfig angle_config;
    Mode_Car_SpinRateConfig spin_rate_config;
    Device_LED_Config led_config;
    Device_RGB_Config rgb_config;
    Device_Servo_Config servo_config;

    if (context != NULL) {
        *context = (Task_Hex25E_Context){0};
    }
    if ((context == NULL) || !Hex25E_Map_ValidateShortestRoute() ||
        !Device_Encoder_MakeConfig(
            HEX25E_CONFIG_LEFT_ENCODER_INDEX,
            HEX25E_CONFIG_ENCODER_COUNTS_PER_REV,
            HEX25E_CONFIG_WHEEL_CIRCUMFERENCE_MM,
            HEX25E_CONFIG_LEFT_ENCODER_DIRECTION, &encoder_config[0]) ||
        !Device_Encoder_MakeConfig(
            HEX25E_CONFIG_RIGHT_ENCODER_INDEX,
            HEX25E_CONFIG_ENCODER_COUNTS_PER_REV,
            HEX25E_CONFIG_WHEEL_CIRCUMFERENCE_MM,
            HEX25E_CONFIG_RIGHT_ENCODER_DIRECTION, &encoder_config[1]) ||
        !Device_Motor_DefaultConfig(
            HEX25E_CONFIG_LEFT_MOTOR_INDEX, &motor_config[0]) ||
        !Device_Motor_DefaultConfig(
            HEX25E_CONFIG_RIGHT_MOTOR_INDEX, &motor_config[1]) ||
        !Device_Servo_DefaultConfig(
            HEX25E_CONFIG_SERVO_INDEX, &servo_config)) {
        return false;
    }
    servo_config.minimum_pulse_us =
        HEX25E_CONFIG_SERVO_MIN_PULSE_US;
    servo_config.maximum_pulse_us =
        HEX25E_CONFIG_SERVO_MAX_PULSE_US;
    servo_config.maximum_angle_degrees =
        HEX25E_CONFIG_SERVO_MAX_ANGLE_DEG;
    if (!Device_Encoder_Init(&context->encoder[0], &encoder_config[0])) {
        return false;
    }
    if (!Device_Encoder_Init(&context->encoder[1], &encoder_config[1])) {
        return false;
    }
    if (!Device_Motor_Init(&context->motor[0], &motor_config[0])) {
        return false;
    }
    if (!Device_Motor_Init(&context->motor[1], &motor_config[1])) {
        return false;
    }
    if (!Device_Servo_Init(&context->servo, &servo_config)) {
        return false;
    }

    speed_config = (Mode_Car_SpeedControllerConfig){
        .low_speed = {
            .kp = HEX25E_CONFIG_SPEED_LOW_KP,
            .ki = HEX25E_CONFIG_SPEED_LOW_KI,
            .kd = HEX25E_CONFIG_SPEED_LOW_KD,
        },
        .high_speed = {
            .kp = HEX25E_CONFIG_SPEED_HIGH_KP,
            .ki = HEX25E_CONFIG_SPEED_HIGH_KI,
            .kd = HEX25E_CONFIG_SPEED_HIGH_KD,
        },
        .high_speed_threshold =
            HEX25E_CONFIG_SPEED_HIGH_THRESHOLD_MM_S,
        .error_deadband = HEX25E_CONFIG_SPEED_ERROR_DEADBAND_MM_S,
        .feedforward_percent_per_speed =
            HEX25E_CONFIG_SPEED_FEEDFORWARD_PERCENT,
        .target_smoothing_time_constant_seconds =
            HEX25E_CONFIG_SPEED_SMOOTHING_TIME_S,
        .maximum_target_speed =
            HEX25E_CONFIG_SPEED_MAXIMUM_TARGET_MM_S,
        .maximum_effort_percent =
            HEX25E_CONFIG_SPEED_MAXIMUM_EFFORT_PERCENT,
        .stopped_speed_threshold =
            HEX25E_CONFIG_SPEED_STOPPED_THRESHOLD_MM_S,
    };
    angle_config = (Mode_Car_AngleControllerConfig){
        .kp = HEX25E_CONFIG_ANGLE_KP,
        .ki = HEX25E_CONFIG_ANGLE_KI,
        .kd = HEX25E_CONFIG_ANGLE_KD,
        .integral_speed_limit =
            HEX25E_CONFIG_ANGLE_INTEGRAL_SPEED_LIMIT,
        .maximum_wheel_speed =
            HEX25E_CONFIG_ANGLE_MAXIMUM_WHEEL_SPEED_MM_S,
        .angle_tolerance_deg =
            HEX25E_CONFIG_ANGLE_TOLERANCE_DEG,
        .rate_tolerance_deg_per_second =
            HEX25E_CONFIG_ANGLE_RATE_TOLERANCE_DPS,
    };
    spin_rate_config = (Mode_Car_SpinRateConfig){
        .kp = HEX25E_CONFIG_TURN_RATE_KP,
        .ki = HEX25E_CONFIG_TURN_RATE_KI,
        .integral_output_limit =
            HEX25E_CONFIG_TURN_RATE_INTEGRAL_LIMIT_MM_S,
        .maximum_correction_drive =
            HEX25E_CONFIG_TURN_RATE_CORRECTION_LIMIT_MM_S,
        .feedforward_drive_per_dps =
            HEX25E_CONFIG_TURN_RATE_FEEDFORWARD_MM_PER_DPS,
        .maximum_drive =
            HEX25E_CONFIG_ANGLE_MAXIMUM_WHEEL_SPEED_MM_S,
    };
    if (!Mode_Car_SpeedController_Init(
            &context->speed_controller, &speed_config) ||
        !Mode_Car_AngleController_Init(
            &context->angle_controller, &angle_config) ||
        !Mode_Car_SpinRate_Init(
            &context->spin_rate_controller, &spin_rate_config)) {
        return false;
    }

    context->state = TASK_HEX25E_STATE_STOPPED;
    context->fault = TASK_HEX25E_FAULT_NONE;
    context->state_started_ms = now_ms;
    context->indicator_led_available[0] =
        Device_LED_DefaultConfig(
            HEX25E_CONFIG_RUN_LED_INDEX, &led_config) &&
        Device_LED_Init(&context->indicator_led[0], &led_config);
    context->indicator_led_available[1] =
        Device_LED_DefaultConfig(
            HEX25E_CONFIG_STATUS_LED_INDEX, &led_config) &&
        Device_LED_Init(&context->indicator_led[1], &led_config);
    context->indicator_rgb_available =
        Device_RGB_DefaultConfig(
            HEX25E_CONFIG_RGB_INDEX, &rgb_config) &&
        Device_RGB_Init(&context->indicator_rgb, &rgb_config);
    update_indicators(context);
    context->initialized = true;
    return true;
}

bool Task_Hex25E_Start(Task_Hex25E_Context *context, uint32_t now_ms)
{
    Device_Encoder_Sample encoder[2];

    if ((context == NULL) || !context->initialized ||
        (context->state == TASK_HEX25E_STATE_FAULT) ||
        !Device_Encoder_GetSample(&context->encoder[0], &encoder[0]) ||
        !Device_Encoder_GetSample(&context->encoder[1], &encoder[1]) ||
        (Device_Motor_SetUnlocked(&context->motor[0], true) !=
            DEVICE_MOTOR_RESULT_OK)) {
        return false;
    }
    if (Device_Motor_SetUnlocked(&context->motor[1], true) !=
        DEVICE_MOTOR_RESULT_OK) {
        (void)Device_Motor_SetUnlocked(&context->motor[0], false);
        return false;
    }
    context->motors_enabled = true;
    context->distance_snapshot_mm = average_distance(encoder);
    context->previous_average_distance_mm = context->distance_snapshot_mm;
    context->route_odometry_mm = 0.0F;
    context->has_previous_average_distance = true;
    context->route_index = 0U;
    context->visited_y_mask = 0U;
    context->heading_letter_mask = 0U;
    context->heading_letter_count = 0U;
    context->current_heading_letter = '\0';
    for (size_t index = 0U;
         index < TASK_HEX25E_HEADING_LOG_CAPACITY; ++index) {
        context->heading_letter_log[index] = '\0';
    }
    context->route_heading_ready = false;
    context->straight_distance_reset_pending = false;
    context->straight_motion_last_cycle = false;
    context->state = TASK_HEX25E_STATE_EXIT_NODE;
    context->state_started_ms = now_ms;
    update_indicators(context);
    return true;
}

bool Task_Hex25E_Update(
    Task_Hex25E_Context *context,
    uint32_t now_ms,
    float dt_seconds,
    const Task_Hex25E_Attitude *attitude,
    Task_Hex25E_Status *status)
{
    const Hex25E_NodeId *route;
    size_t route_count;
    Device_Encoder_Sample encoder[2];
    float left_target = 0.0F;
    float right_target = 0.0F;
    float distance;
    bool moving = false;
    bool straight_motion_commanded = false;

    if ((status != NULL) && (context != NULL)) {
        make_status(context, now_ms, status);
    }
    if ((context == NULL) || !context->initialized || (status == NULL) ||
        !isfinite(dt_seconds) || (dt_seconds <= 0.0F)) {
        return false;
    }
    route = Hex25E_Map_GetShortestRoute(&route_count);
    if ((route == NULL) || (context->route_index >= route_count) ||
        !get_encoder_samples(context, dt_seconds, encoder)) {
        enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
        make_status(context, now_ms, status);
        return false;
    }
    distance = average_distance(encoder);
    if (!context->has_previous_average_distance) {
        context->previous_average_distance_mm = distance;
        context->has_previous_average_distance = true;
    }
    else {
        float delta_distance =
            distance - context->previous_average_distance_mm;
        context->previous_average_distance_mm = distance;
        if (context->straight_motion_last_cycle &&
            !context->straight_distance_reset_pending) {
            context->route_odometry_mm += fabsf(delta_distance);
        }
    }
    context->left_measured_speed =
        encoder[0].speed_distance_per_second;
    context->right_measured_speed =
        encoder[1].speed_distance_per_second;
    context->left_cumulative_counts = encoder[0].cumulative_counts;
    context->right_cumulative_counts = encoder[1].cumulative_counts;
    context->left_cumulative_distance_mm =
        encoder[0].cumulative_distance;
    context->right_cumulative_distance_mm =
        encoder[1].cumulative_distance;

    switch (context->state) {
    case TASK_HEX25E_STATE_STOPPED:
        break;

    case TASK_HEX25E_STATE_EXIT_NODE:
        if (context->straight_distance_reset_pending) {
            /* A turn changes both cumulative wheel distances even though the
             * chassis centre should not advance.  Start the next map edge
             * from the first encoder sample after the turn has been braked,
             * so wheel asymmetry cannot leak into straight-line odometry. */
            context->distance_snapshot_mm = distance;
            context->straight_distance_reset_pending = false;
        }
        if (!context->route_heading_ready) {
            if (!valid_attitude(attitude)) {
                enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
                break;
            }
            context->angle_target_yaw_deg = attitude->yaw_deg;
            context->route_heading_ready = true;
        }
        if (!make_straight_targets(context, attitude,
                &left_target, &right_target)) {
            enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
            break;
        }
        moving = true;
        straight_motion_commanded = true;
        context->state = TASK_HEX25E_STATE_DRIVE_EDGE;
        break;

    case TASK_HEX25E_STATE_DRIVE_EDGE: {
        size_t target_route_index;
        float nominal_leg_mm;
        const float raw_leg_distance =
            fabsf(distance - context->distance_snapshot_mm);

        if (context->route_index + 1U >= route_count) {
            enter_fault(context, TASK_HEX25E_FAULT_ROUTE);
            break;
        }
        target_route_index = context->route_index + 1U;
        if (!Hex25E_Map_GetRouteEdgeLength(
                context->route_index, &nominal_leg_mm)) {
            enter_fault(context, TASK_HEX25E_FAULT_ROUTE);
            break;
        }
        if (!make_straight_targets(context, attitude,
                &left_target, &right_target)) {
            enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
            break;
        }
        moving = true;
        straight_motion_commanded = true;
        if (raw_leg_distance >= nominal_leg_mm) {
            context->route_index = target_route_index;
            context->distance_snapshot_mm = distance;
            context->state = TASK_HEX25E_STATE_CENTER_NODE;
        }
        break;
    }

    case TASK_HEX25E_STATE_CENTER_NODE: {
        Hex25E_NodeId arrived = route[context->route_index];

        if (context->route_index == route_count - 1U) {
            if ((context->visited_y_mask & g_all_y_mask) != g_all_y_mask) {
                enter_fault(context, TASK_HEX25E_FAULT_ROUTE);
            }
            else {
                context->state = TASK_HEX25E_STATE_DONE;
                context->motors_enabled = false;
                (void)Device_Motor_SetUnlocked(&context->motor[0], false);
                (void)Device_Motor_SetUnlocked(&context->motor[1], false);
                moving = false;
            }
        }
        else if (Hex25E_Map_IsYJunction(arrived) &&
            ((context->visited_y_mask &
                (UINT32_C(1) << (unsigned int)arrived)) == 0U)) {
            context->visited_y_mask |=
                UINT32_C(1) << (unsigned int)arrived;
            context->state_started_ms = now_ms;
            context->state = TASK_HEX25E_STATE_DWELL_Y;
            moving = false;
        }
        else if (!valid_attitude(attitude)) {
            enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
        }
        else if (!start_turn(context, now_ms, false,
                attitude->yaw_deg)) {
            enter_fault(context, TASK_HEX25E_FAULT_ROUTE);
        }
        break;
    }

    case TASK_HEX25E_STATE_DWELL_Y:
        if ((uint32_t)(now_ms - context->state_started_ms) >=
            HEX25E_CONFIG_Y_DWELL_MS) {
            if (!valid_attitude(attitude)) {
                enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
            }
            else if (!start_turn(context, now_ms, true,
                    attitude->yaw_deg)) {
                enter_fault(context, TASK_HEX25E_FAULT_ROUTE);
            }
        }
        break;

    case TASK_HEX25E_STATE_TURN: {
        Mode_Car_AngleControllerInput angle_input;
        Mode_Car_AngleControllerOutput angle_output;
        Mode_Car_SpinRateInput spin_input;
        Mode_Car_Output spin_output;

        if (!valid_attitude(attitude)) {
            enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
            break;
        }
        if (context->turn_rate_active) {
            if (context->turn_intermediate_pause_active) {
                context->turn_previous_yaw_deg = attitude->yaw_deg;
                if ((uint32_t)(now_ms - context->state_started_ms) <
                    HEX25E_CONFIG_U_TURN_INTERMEDIATE_DWELL_MS) {
                    moving = false;
                    break;
                }
                context->turn_intermediate_pause_active = false;
                Mode_Car_SpinRate_Reset(
                    &context->spin_rate_controller);
            }
            float yaw_delta = wrap_yaw(
                attitude->yaw_deg - context->turn_previous_yaw_deg);
            float directed_delta = context->turn_direction * yaw_delta;

            context->turn_previous_yaw_deg = attitude->yaw_deg;
            context->turn_accumulated_deg += directed_delta;
            if (context->turn_accumulated_deg < 0.0F) {
                context->turn_accumulated_deg = 0.0F;
            }
            record_current_heading_letter(context);
            if (context->turn_pause_after_full_rotation &&
                (context->turn_accumulated_deg >= 360.0F)) {
                context->turn_pause_after_full_rotation = false;
                context->turn_intermediate_pause_active = true;
                context->turn_accumulated_deg = 360.0F;
                record_current_heading_letter(context);
                context->state_started_ms = now_ms;
                Mode_Car_SpinRate_Reset(
                    &context->spin_rate_controller);
                moving = false;
                break;
            }
            if (context->turn_accumulated_deg <
                context->turn_target_travel_deg) {
                spin_input = (Mode_Car_SpinRateInput){
                    .time = {
                        .now_ms = now_ms,
                        .dt_seconds = dt_seconds,
                    },
                    .target_yaw_rate_deg_per_second =
                        context->turn_direction *
                            HEX25E_CONFIG_TURN_RATE_TARGET_DPS,
                    .measured_yaw_rate_deg_per_second =
                        attitude->yaw_rate_deg_per_second,
                };
                if (!Mode_Car_SpinRate_Update(
                        &context->spin_rate_controller,
                        &spin_input, &spin_output)) {
                    enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
                    break;
                }
                left_target = spin_output.left_drive;
                right_target = spin_output.right_drive;
                moving = spin_output.motion_requested;
                break;
            }

            /* Brake once after the continuous rate-controlled sweep.  The
             * normal angle loop then removes only the small stop error. */
            context->turn_rate_active = false;
            context->angle_stable_timing = false;
            Mode_Car_SpinRate_Reset(&context->spin_rate_controller);
            Mode_Car_AngleController_Reset(&context->angle_controller);
            moving = false;
            break;
        }
        angle_input = (Mode_Car_AngleControllerInput){
            .dt_seconds = dt_seconds,
            .target_yaw_deg = context->angle_target_yaw_deg,
            .measured_yaw_deg = attitude->yaw_deg,
            .measured_yaw_rate_deg_per_second =
                attitude->yaw_rate_deg_per_second,
        };
        if (!Mode_Car_AngleController_Update(
                &context->angle_controller, &angle_input, &angle_output)) {
            enter_fault(context, TASK_HEX25E_FAULT_SENSOR);
            break;
        }
        left_target = angle_output.target_left_speed;
        right_target = angle_output.target_right_speed;
        moving = true;
        if (angle_output.within_tolerance) {
            if (!context->angle_stable_timing) {
                context->angle_stable_timing = true;
                context->angle_stable_started_ms = now_ms;
            }
            if ((uint32_t)(now_ms - context->angle_stable_started_ms) >=
                HEX25E_CONFIG_ANGLE_STABLE_MS) {
                context->angle_stable_timing = false;
                context->straight_distance_reset_pending = true;
                context->current_heading_letter = '\0';
                context->state = TASK_HEX25E_STATE_EXIT_NODE;
                moving = false;
                left_target = 0.0F;
                right_target = 0.0F;
            }
        }
        else {
            context->angle_stable_timing = false;
        }
        break;
    }

    case TASK_HEX25E_STATE_DONE:
        context->motors_enabled = false;
        break;

    case TASK_HEX25E_STATE_FAULT:
    default:
        break;
    }

    if (context->state == TASK_HEX25E_STATE_FAULT) {
        make_status(context, now_ms, status);
        return false;
    }
    context->left_target_speed = left_target;
    context->right_target_speed = right_target;
    if (moving && context->motors_enabled) {
        if (!apply_wheel_targets(context, left_target, right_target,
                encoder, dt_seconds)) {
            enter_fault(context, TASK_HEX25E_FAULT_MOTOR);
            make_status(context, now_ms, status);
            return false;
        }
    }
    else if (!stop_motors(context)) {
        enter_fault(context, TASK_HEX25E_FAULT_MOTOR);
        make_status(context, now_ms, status);
        return false;
    }

    context->straight_motion_last_cycle = moving &&
        straight_motion_commanded && context->motors_enabled;

    update_indicators(context);
    make_status(context, now_ms, status);
    return true;
}

void Task_Hex25E_Stop(Task_Hex25E_Context *context)
{
    if ((context == NULL) || !context->initialized) {
        return;
    }
    context->motors_enabled = false;
    context->straight_motion_last_cycle = false;
    context->state = TASK_HEX25E_STATE_STOPPED;
    (void)stop_motors(context);
    (void)Device_Motor_SetUnlocked(&context->motor[0], false);
    (void)Device_Motor_SetUnlocked(&context->motor[1], false);
    (void)Device_Servo_Enable(&context->servo, false);
    update_indicators(context);
}
