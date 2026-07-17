#include "task_hex25e_comm.h"
#include "hex25e_config.h"
#include "task_hex25e_vofa.h"

#include "device_serial.h"

#include <stddef.h>

typedef enum {
    HEX25E_VOFA_IMU_YAW_DEG = 0,
    HEX25E_VOFA_ODO_ROUTE_MM,
    HEX25E_VOFA_ROUTE_NEXT_NODE,
    HEX25E_VOFA_WHEEL_ACTUAL_LEFT_MM_S,
    HEX25E_VOFA_WHEEL_ACTUAL_RIGHT_MM_S,
    HEX25E_VOFA_FAULT_CODE,
    HEX25E_VOFA_CHANNEL_COUNT,
} Task_Hex25E_VofaChannel;

_Static_assert(
    HEX25E_VOFA_CHANNEL_COUNT <= HEX25E_CONFIG_VOFA_MAX_CHANNELS,
    "Hex25E VOFA channel count exceeds task encoder capacity");

static const char g_vofa_csv_header[] =
    "IMU.yaw_deg,ODO.route_mm,ROUTE.next_node,"
    "WHEEL.actual_l_mm_s,WHEEL.actual_r_mm_s,FAULT.code\r\n";

typedef struct {
    Device_Serial bluetooth;
    Task_Hex25E_Vofa vofa;
    char line[HEX25E_CONFIG_COMM_RX_LINE_SIZE];
    size_t line_length;
    Task_Hex25E_Command command[HEX25E_CONFIG_COMMAND_QUEUE_SIZE];
    size_t command_head;
    size_t command_tail;
    uint32_t last_rx_ms;
    uint32_t tx_failure_started_ms;
    bool line_overflow;
    bool line_pending;
    bool vofa_header_sent;
    bool tx_failure_timing;
    bool initialized;
} Task_Hex25E_CommContext;

static Task_Hex25E_CommContext g_comm;

static bool handle_vofa_tx_result(uint32_t now_ms, bool sent)
{
    if (sent) {
        g_comm.tx_failure_timing = false;
        return true;
    }
    if (!g_comm.tx_failure_timing) {
        g_comm.tx_failure_started_ms = now_ms;
        g_comm.tx_failure_timing = true;
        return false;
    }
    if ((uint32_t)(now_ms - g_comm.tx_failure_started_ms) <
        HEX25E_CONFIG_BLUETOOTH_TX_STALL_MS) {
        return false;
    }

    /* A continuously full TX ring means the interrupt/DMA path is no longer
     * making progress. Abort the stuck frame, restart the UART backend and
     * force a fresh FireWater header on the next telemetry period. */
    if (!Device_Serial_Recover(&g_comm.bluetooth) ||
        !Task_Hex25E_Vofa_Init(&g_comm.vofa, &g_comm.bluetooth)) {
        return false;
    }
    g_comm.line_length = 0U;
    g_comm.line_overflow = false;
    g_comm.line_pending = false;
    g_comm.command_head = 0U;
    g_comm.command_tail = 0U;
    g_comm.vofa_header_sent = false;
    g_comm.tx_failure_timing = false;
    return false;
}

static char ascii_upper(char value)
{
    return ((value >= 'a') && (value <= 'z')) ?
        (char)(value - ('a' - 'A')) : value;
}

static bool line_equals(const char *line, size_t length, const char *word)
{
    size_t index = 0U;

    while ((index < length) && (word[index] != '\0')) {
        if (ascii_upper(line[index]) != word[index]) {
            return false;
        }
        ++index;
    }
    return (index == length) && (word[index] == '\0');
}

static void queue_command(Task_Hex25E_Command command)
{
    size_t next = (g_comm.command_head + 1U) %
        HEX25E_CONFIG_COMMAND_QUEUE_SIZE;

    if (next == g_comm.command_tail) {
        if (command != TASK_HEX25E_COMMAND_STOP) {
            return;
        }
        g_comm.command_head = 0U;
        g_comm.command_tail = 0U;
        next = 1U;
    }
    g_comm.command[g_comm.command_head] = command;
    g_comm.command_head = next;
}

static void finish_line(void)
{
    size_t begin = 0U;
    size_t end = g_comm.line_length;
    Task_Hex25E_Command command = TASK_HEX25E_COMMAND_UNKNOWN;

    while ((begin < end) && ((g_comm.line[begin] == ' ') ||
        (g_comm.line[begin] == '\t'))) {
        ++begin;
    }
    while ((end > begin) && ((g_comm.line[end - 1U] == ' ') ||
        (g_comm.line[end - 1U] == '\t'))) {
        --end;
    }
    if (!g_comm.line_overflow && (end > begin)) {
        const char *line = &g_comm.line[begin];
        const size_t length = end - begin;

        if (line_equals(line, length, "START")) {
            command = TASK_HEX25E_COMMAND_START;
        }
        else if (line_equals(line, length, "STOP")) {
            command = TASK_HEX25E_COMMAND_STOP;
        }
        else if (line_equals(line, length, "STATUS")) {
            command = TASK_HEX25E_COMMAND_STATUS;
            g_comm.vofa_header_sent = false;
        }
        queue_command(command);
    }
    else if (g_comm.line_overflow) {
        queue_command(TASK_HEX25E_COMMAND_UNKNOWN);
    }
    g_comm.line_length = 0U;
    g_comm.line_overflow = false;
    g_comm.line_pending = false;
}

bool Task_Hex25E_Comm_Init(void)
{
    const Device_Serial_Config bluetooth_config = {
        .transport_index = HEX25E_CONFIG_BLUETOOTH_TRANSPORT_INDEX,
        .expected_baud_rate = HEX25E_CONFIG_BLUETOOTH_BAUD_RATE,
        .minimum_tx_capacity = HEX25E_CONFIG_VOFA_MIN_TX_CAPACITY,
        .minimum_rx_capacity = 1U,
        .require_buffered_tx = true,
        .require_buffered_rx = true,
    };
    Task_Hex25E_CommContext next = {0};

    if (!Device_Serial_Init(&next.bluetooth, &bluetooth_config)) {
        g_comm = (Task_Hex25E_CommContext){0};
        return false;
    }
    g_comm = next;
    if (!Task_Hex25E_Vofa_Init(&g_comm.vofa, &g_comm.bluetooth)) {
        g_comm = (Task_Hex25E_CommContext){0};
        return false;
    }
    g_comm.initialized = true;
    return true;
}

bool Task_Hex25E_Comm_PollCommand(
    uint32_t now_ms, Task_Hex25E_Command *command)
{
    uint8_t bytes[HEX25E_CONFIG_COMM_RX_CHUNK_SIZE];
    Device_Serial_RxStatus rx_status;
    size_t received = 0U;

    if (command != NULL) {
        *command = TASK_HEX25E_COMMAND_NONE;
    }
    if (!g_comm.initialized || (command == NULL)) {
        return false;
    }
    if (!Device_Serial_GetRxStatus(&g_comm.bluetooth, &rx_status)) {
        return false;
    }
    if (rx_status.sticky_errors != DEVICE_SERIAL_RX_ERROR_NONE) {
        if (!Device_Serial_ClearRxStatus(&g_comm.bluetooth)) {
            return false;
        }
        do {
            if (Device_Serial_ReadAvailable(&g_comm.bluetooth, bytes,
                    sizeof(bytes), &received) != DEVICE_SERIAL_RESULT_OK) {
                return false;
            }
        } while (received == sizeof(bytes));
        g_comm.line_length = 0U;
        g_comm.line_overflow = false;
        g_comm.line_pending = false;
        g_comm.command_head = 0U;
        g_comm.command_tail = 0U;
        /* A malformed/noisy frame is not an authenticated STOP command.
         * Drop it silently so diagnostics never corrupt FireWater. */
        return true;
    }
    if (Device_Serial_ReadAvailable(
            &g_comm.bluetooth, bytes, sizeof(bytes), &received) !=
        DEVICE_SERIAL_RESULT_OK) {
        return false;
    }
    if (received > 0U) {
        g_comm.last_rx_ms = now_ms;
        g_comm.line_pending = true;
    }
    for (size_t index = 0U; index < received; ++index) {
        const char value = (char)bytes[index];

        if ((value == '\r') || (value == '\n')) {
            if ((g_comm.line_length > 0U) || g_comm.line_overflow) {
                finish_line();
            }
        }
        else if (!g_comm.line_overflow) {
            if (g_comm.line_length < sizeof(g_comm.line)) {
                g_comm.line[g_comm.line_length++] = value;
            }
            else {
                g_comm.line_overflow = true;
            }
        }
    }
    if (g_comm.line_pending &&
        ((uint32_t)(now_ms - g_comm.last_rx_ms) >=
            HEX25E_CONFIG_RX_IDLE_TERMINATE_MS)) {
        finish_line();
    }
    if (g_comm.command_tail != g_comm.command_head) {
        *command = g_comm.command[g_comm.command_tail];
        g_comm.command_tail = (g_comm.command_tail + 1U) %
            HEX25E_CONFIG_COMMAND_QUEUE_SIZE;
    }
    return true;
}

bool Task_Hex25E_Comm_SendVofa(
    const Task_Hex25E_Telemetry *telemetry)
{
    float channels[HEX25E_VOFA_CHANNEL_COUNT];

    if (!g_comm.initialized || (telemetry == NULL)) {
        return false;
    }
    channels[HEX25E_VOFA_IMU_YAW_DEG] =
        (telemetry->imu_online && telemetry->imu_has_euler) ?
            telemetry->euler_deg[2] : -9999.0F;
    channels[HEX25E_VOFA_ODO_ROUTE_MM] =
        telemetry->route_odometry_mm;
    channels[HEX25E_VOFA_ROUTE_NEXT_NODE] = (float)telemetry->next_node;
    channels[HEX25E_VOFA_WHEEL_ACTUAL_LEFT_MM_S] =
        telemetry->left_measured_speed;
    channels[HEX25E_VOFA_WHEEL_ACTUAL_RIGHT_MM_S] =
        telemetry->right_measured_speed;
    channels[HEX25E_VOFA_FAULT_CODE] = (float)telemetry->fault;

    /* Follow the proven jixun3 VOFA path: one CSV header followed by a fixed
     * numeric FireWater row. STATUS clears header_sent so a newly connected
     * Bluetooth client can explicitly request the categorized header again. */
    if (!g_comm.vofa_header_sent) {
        if (!Task_Hex25E_Vofa_SendText(
                &g_comm.vofa, g_vofa_csv_header)) {
            return handle_vofa_tx_result(telemetry->now_ms, false);
        }
        g_comm.vofa_header_sent = true;
    }
    return handle_vofa_tx_result(telemetry->now_ms,
        Task_Hex25E_Vofa_SendCsvFloat(
            &g_comm.vofa, channels, HEX25E_VOFA_CHANNEL_COUNT));
}
