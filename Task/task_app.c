#include "task_app.h"

#include "FreeRTOS.h"
#include "hex25e_config.h"
#include "task.h"

#if defined(TEMPLATE_TASK_HEX25E)
#include "device_key_input.h"
#include "task_hex25e.h"
#include "task_hex25e_comm.h"
#include "task_hex25e_imu.h"
#include "task_hex25e_vision.h"
static Task_Hex25E_Context g_TASK_Hex25E;
static Task_Hex25E_Status g_TASK_Hex25EStatus;
static uint32_t g_TASK_LastVofaMs;
static Device_JY61P_Snapshot g_TASK_Hex25EIMUSnapshot;
static Device_KeyInput g_TASK_Hex25EUserKey;
static bool g_TASK_Hex25EIMUReady;
static bool g_TASK_Hex25EVisionReady;
#endif

static StaticTask_t g_TASK_ControlTCB;
static StackType_t
    g_TASK_ControlStack[HEX25E_CONFIG_CONTROL_STACK_DEPTH];

volatile uint32_t g_TASK_Heartbeat = 0U;
volatile uint32_t g_TASK_Fault = 0U;
volatile uint32_t g_TASK_Ready = 0U;
volatile uint32_t g_TASK_MotorLocked = 1U;
volatile uint32_t g_TASK_OutputEnabled = 0U;
volatile uint32_t g_TASK_ModeFault = 0U;
volatile uint32_t g_TASK_UpdateCount = 0U;
volatile uint32_t g_TASK_NowMs = 0U;
volatile uint32_t g_TASK_RouteState = 0U;
volatile uint32_t g_TASK_CurrentNode = 0U;
volatile uint32_t g_TASK_NextNode = 0U;
volatile uint32_t g_TASK_VisitedYMask = 0U;
volatile uint32_t g_TASK_DwellRemainingMs = 0U;
volatile uint32_t g_TASK_UserKeyPressed = 0U;
volatile uint32_t g_TASK_UserKeyPressCount = 0U;
volatile uint32_t g_TASK_HeadingLetter = 0U;
volatile uint32_t g_TASK_HeadingLetterMask = 0U;
volatile uint32_t g_TASK_HeadingLetterCount = 0U;
volatile uint32_t g_TASK_VisionFault = 0U;
volatile uint32_t g_TASK_VisionGeneration = 0U;
volatile uint32_t g_TASK_VisionColor = 0U;
volatile uint32_t g_TASK_VisionShape = 0U;
volatile uint8_t
    g_TASK_HeadingLetterLog[TASK_HEADING_LETTER_LOG_CAPACITY] = {0U};

#if defined(TEMPLATE_TASK_HEX25E)
static Task_Hex25E_Telemetry Task_MakeHex25ETelemetry(
    const Task_Hex25E_Status *status)
{
    const Task_Hex25E_Telemetry telemetry = {
        .now_ms = g_TASK_NowMs,
        .state = (uint32_t)status->state,
        .fault = (uint32_t)status->fault,
        .current_node = (uint32_t)status->current_node,
        .next_node = (uint32_t)status->next_node,
        .visited_y_mask = status->visited_y_mask,
        .dwell_remaining_ms = status->dwell_remaining_ms,
        .motors_enabled = status->motors_enabled,
        .user_key_pressed = g_TASK_UserKeyPressed != 0U,
        .user_key_press_count = g_TASK_UserKeyPressCount,
        .left_target_speed = status->left_target_speed,
        .right_target_speed = status->right_target_speed,
        .left_measured_speed = status->left_measured_speed,
        .right_measured_speed = status->right_measured_speed,
        .left_cumulative_counts = status->left_cumulative_counts,
        .right_cumulative_counts = status->right_cumulative_counts,
        .left_cumulative_distance_mm =
            status->left_cumulative_distance_mm,
        .right_cumulative_distance_mm =
            status->right_cumulative_distance_mm,
        .route_odometry_mm = status->route_odometry_mm,
        .left_motor_command = status->left_motor_command,
        .right_motor_command = status->right_motor_command,
        .angular_rate_dps = {
            g_TASK_Hex25EIMUSnapshot.angular_rate_dps[0],
            g_TASK_Hex25EIMUSnapshot.angular_rate_dps[1],
            g_TASK_Hex25EIMUSnapshot.angular_rate_dps[2],
        },
        .euler_deg = {
            g_TASK_Hex25EIMUSnapshot.euler_deg[0],
            g_TASK_Hex25EIMUSnapshot.euler_deg[1],
            g_TASK_Hex25EIMUSnapshot.euler_deg[2],
        },
        .imu_online = g_TASK_Hex25EIMUSnapshot.online,
        .imu_has_angular_rate =
            g_TASK_Hex25EIMUSnapshot.has_angular_rate,
        .imu_has_euler = g_TASK_Hex25EIMUSnapshot.has_euler,
    };

    return telemetry;
}

static bool Task_Hex25E_StartRoute(void)
{
    Task_Hex25E_Stop(&g_TASK_Hex25E);
    if (!Task_Hex25E_Init(&g_TASK_Hex25E, g_TASK_NowMs) ||
        !Task_Hex25E_Start(&g_TASK_Hex25E, g_TASK_NowMs)) {
        g_TASK_Fault = 1U;
        return false;
    }
    g_TASK_Fault = 0U;
    return true;
}

static void Task_Hex25E_StopRoute(void)
{
    Task_Hex25E_Stop(&g_TASK_Hex25E);
}

static void Task_RunHex25EOnce(void)
{
    Task_Hex25E_Command command;
    Task_Hex25E_Attitude attitude;
    bool user_key_pressed = false;
    bool status_requested = false;
    bool command_ok = true;
    Task_Hex25E_VisionStatus vision_status;

    if (g_TASK_Hex25EVisionReady) {
        if (!Task_Hex25E_Vision_Poll(g_TASK_NowMs) ||
            !Task_Hex25E_Vision_GetStatus(&vision_status)) {
            g_TASK_VisionFault = 1U;
        }
        else {
            g_TASK_VisionFault = vision_status.fault ? 1U : 0U;
            if (vision_status.has_result) {
                g_TASK_VisionGeneration = vision_status.generation;
                g_TASK_VisionColor = (uint32_t)vision_status.color;
                g_TASK_VisionShape = (uint32_t)vision_status.shape;
            }
        }
    }

    if (g_TASK_Hex25EIMUReady) {
        (void)Task_Hex25E_IMU_Update(g_TASK_NowMs);
        (void)Task_Hex25E_IMU_GetSnapshot(
            &g_TASK_Hex25EIMUSnapshot);
    }

    if (!Device_KeyInput_UpdatePressedEdge(&g_TASK_Hex25EUserKey,
            g_TASK_NowMs, &user_key_pressed)) {
        g_TASK_Fault = 1U;
        command_ok = false;
    }
    else if (user_key_pressed) {
        ++g_TASK_UserKeyPressCount;
        if (!HEX25E_CONFIG_ENABLE_MOTION) {
            status_requested = true;
        }
        else if (g_TASK_Hex25EStatus.motors_enabled) {
            Task_Hex25E_StopRoute();
        }
        else if (!Task_Hex25E_StartRoute()) {
            command_ok = false;
        }
    }
    g_TASK_UserKeyPressed =
        Device_Key_IsPressed(&g_TASK_Hex25EUserKey.key) ? 1U : 0U;

    attitude = (Task_Hex25E_Attitude){
        .yaw_deg = g_TASK_Hex25EIMUSnapshot.euler_deg[2],
        .yaw_rate_deg_per_second =
            g_TASK_Hex25EIMUSnapshot.angular_rate_dps[2],
        .valid = g_TASK_Hex25EIMUSnapshot.online &&
            g_TASK_Hex25EIMUSnapshot.has_euler &&
            g_TASK_Hex25EIMUSnapshot.has_angular_rate,
    };

    for (size_t index = 0U; index < 4U; ++index) {
        if (!Task_Hex25E_Comm_PollCommand(g_TASK_NowMs, &command)) {
            g_TASK_Fault = 1U;
            command_ok = false;
            break;
        }
        if (command == TASK_HEX25E_COMMAND_NONE) {
            break;
        }
        if (command == TASK_HEX25E_COMMAND_START) {
            if (HEX25E_CONFIG_ENABLE_MOTION &&
                !Task_Hex25E_StartRoute()) {
                command_ok = false;
            }
        }
        else if (command == TASK_HEX25E_COMMAND_STOP) {
            if (HEX25E_CONFIG_ENABLE_MOTION) {
                Task_Hex25E_StopRoute();
            }
        }
        else if (command == TASK_HEX25E_COMMAND_STATUS) {
            status_requested = true;
        }
    }

    const bool updated = Task_Hex25E_Update(
        &g_TASK_Hex25E, g_TASK_NowMs,
        HEX25E_CONFIG_CONTROL_PERIOD_SECONDS, &attitude,
        &g_TASK_Hex25EStatus);
    const Task_Hex25E_Telemetry telemetry =
        Task_MakeHex25ETelemetry(&g_TASK_Hex25EStatus);

    ++g_TASK_UpdateCount;
    g_TASK_ModeFault = (uint32_t)g_TASK_Hex25EStatus.fault;
    g_TASK_MotorLocked = g_TASK_Hex25EStatus.motors_enabled ? 0U : 1U;
    g_TASK_OutputEnabled =
        g_TASK_Hex25EStatus.motors_enabled ? 1U : 0U;
    g_TASK_RouteState = (uint32_t)g_TASK_Hex25EStatus.state;
    g_TASK_CurrentNode = (uint32_t)g_TASK_Hex25EStatus.current_node;
    g_TASK_NextNode = (uint32_t)g_TASK_Hex25EStatus.next_node;
    g_TASK_VisitedYMask = g_TASK_Hex25EStatus.visited_y_mask;
    g_TASK_DwellRemainingMs = g_TASK_Hex25EStatus.dwell_remaining_ms;
    g_TASK_HeadingLetter =
        (uint32_t)(uint8_t)g_TASK_Hex25EStatus.current_heading_letter;
    g_TASK_HeadingLetterMask = g_TASK_Hex25EStatus.heading_letter_mask;
    g_TASK_HeadingLetterCount = g_TASK_Hex25EStatus.heading_letter_count;
    for (size_t index = 0U;
         index < TASK_HEADING_LETTER_LOG_CAPACITY; ++index) {
        g_TASK_HeadingLetterLog[index] =
            (uint8_t)g_TASK_Hex25EStatus.heading_letter_log[index];
    }
    if (!command_ok || !updated ||
        (g_TASK_Hex25EStatus.fault != TASK_HEX25E_FAULT_NONE)) {
        g_TASK_Fault = 1U;
    }
    if (status_requested || (g_TASK_UpdateCount == 1U) ||
        ((uint32_t)(g_TASK_NowMs - g_TASK_LastVofaMs) >=
            HEX25E_CONFIG_TELEMETRY_PERIOD_MS)) {
        /* Keep a fixed sampling cadence even if one frame cannot be queued.
         * SendVofa exposes the missed attempt through seq/drop. */
        g_TASK_LastVofaMs = g_TASK_NowMs;
        (void)Task_Hex25E_Comm_SendVofa(&telemetry);
    }
}
#endif

static void Task_Control(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        g_TASK_NowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
#if defined(TEMPLATE_TASK_HEX25E)
        if (g_TASK_Ready != 0U) {
            Task_RunHex25EOnce();
        }
#endif
        ++g_TASK_Heartbeat;
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(HEX25E_CONFIG_CONTROL_PERIOD_MS));
    }
}

bool Task_Init(void)
{
    TaskHandle_t task;

    g_TASK_Heartbeat = 0U;
    g_TASK_Fault = 0U;
    g_TASK_Ready = 0U;
    g_TASK_MotorLocked = 1U;
    g_TASK_OutputEnabled = 0U;
    g_TASK_ModeFault = 0U;
    g_TASK_UpdateCount = 0U;
    g_TASK_NowMs = 0U;
    g_TASK_RouteState = 0U;
    g_TASK_CurrentNode = 0U;
    g_TASK_NextNode = 0U;
    g_TASK_VisitedYMask = 0U;
    g_TASK_DwellRemainingMs = 0U;
    g_TASK_UserKeyPressed = 0U;
    g_TASK_UserKeyPressCount = 0U;
    g_TASK_HeadingLetter = 0U;
    g_TASK_HeadingLetterMask = 0U;
    g_TASK_HeadingLetterCount = 0U;
    g_TASK_VisionFault = 0U;
    g_TASK_VisionGeneration = 0U;
    g_TASK_VisionColor = 0U;
    g_TASK_VisionShape = 0U;
    for (size_t index = 0U;
         index < TASK_HEADING_LETTER_LOG_CAPACITY; ++index) {
        g_TASK_HeadingLetterLog[index] = 0U;
    }

#if defined(TEMPLATE_TASK_HEX25E)
    const Device_KeyInput_Config user_key_config = {
        .key = {
            .debounce_ms = HEX25E_CONFIG_KEY_DEBOUNCE_MS,
            .long_press_ms = HEX25E_CONFIG_KEY_LONG_PRESS_MS,
            .double_click_ms = HEX25E_CONFIG_KEY_DOUBLE_CLICK_MS,
        },
        .input_index = HEX25E_CONFIG_KEY_INPUT_INDEX,
    };

    g_TASK_Hex25EStatus = (Task_Hex25E_Status){0};
    g_TASK_LastVofaMs = 0U;
    g_TASK_Hex25EIMUSnapshot = (Device_JY61P_Snapshot){0};
    g_TASK_Hex25EIMUReady = Task_Hex25E_IMU_Init();
    g_TASK_Hex25EVisionReady = Task_Hex25E_Vision_Init();
    if (!Task_Hex25E_Comm_Init() ||
        !g_TASK_Hex25EVisionReady ||
        !Device_KeyInput_Init(
            &g_TASK_Hex25EUserKey, &user_key_config, 0U) ||
        !Task_Hex25E_Init(&g_TASK_Hex25E, 0U)) {
        g_TASK_Fault = 1U;
        return false;
    }
#endif

    task = xTaskCreateStatic(
        Task_Control,
        "Control",
        HEX25E_CONFIG_CONTROL_STACK_DEPTH,
        NULL,
        tskIDLE_PRIORITY + HEX25E_CONFIG_CONTROL_PRIORITY_OFFSET,
        g_TASK_ControlStack,
        &g_TASK_ControlTCB);
    if (task == NULL) {
        g_TASK_Fault = 2U;
        return false;
    }

    g_TASK_Ready = 1U;
    return true;
}
