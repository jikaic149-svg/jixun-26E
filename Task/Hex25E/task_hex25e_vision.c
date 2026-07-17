#include "task_hex25e_vision.h"

#include "device_serial.h"
#include "hex25e_config.h"

#include <stddef.h>

typedef struct {
    Device_Serial serial;
    Device_Vision vision;
    Device_Vision_Config vision_config;
    Task_Hex25E_VisionStatus status;
    bool initialized;
} Task_Hex25E_VisionContext;

static Task_Hex25E_VisionContext g_vision;

static bool write_frame(const uint8_t *bytes, size_t length)
{
    const Device_Serial_Result result = Device_Serial_WriteBuffered(
        &g_vision.serial, bytes, length);

    /* A full TX ring may drop this ACK safely: the remote critical frame will
     * retry and Device_Vision will generate a duplicate ACK next time. */
    return (result == DEVICE_SERIAL_RESULT_OK) ||
        (result == DEVICE_SERIAL_RESULT_BUSY);
}

static bool recover_link(void)
{
    if (!Device_Serial_Recover(&g_vision.serial) ||
        !Device_Vision_Init(
            &g_vision.vision, &g_vision.vision_config)) {
        g_vision.status.fault = true;
        return false;
    }
    if (g_vision.status.recovery_count != UINT32_MAX) {
        ++g_vision.status.recovery_count;
    }
    g_vision.status.fault = false;
    return true;
}

static bool feed_received_byte(uint32_t now_ms, uint8_t byte)
{
    uint8_t ack[DEVICE_VISION_ACK_FRAME_MAX_SIZE];
    size_t ack_length = 0U;
    const Device_Vision_FeedResult result = Device_Vision_FeedByte(
        &g_vision.vision, now_ms, byte,
        ack, sizeof(ack), &ack_length);

    switch (result) {
    case DEVICE_VISION_FEED_CRITICAL_EVENT_AND_ACK_READY:
    case DEVICE_VISION_FEED_DUPLICATE_ACK_READY:
        /* The buffered UART copies the complete frame before returning. */
        return (ack_length > 0U) && write_frame(ack, ack_length);
    case DEVICE_VISION_FEED_ACK_OUTPUT_TOO_SMALL:
    case DEVICE_VISION_FEED_INVALID_ARGUMENT:
        return false;
    default:
        return true;
    }
}

static bool pump_critical_tx(uint32_t now_ms)
{
    Device_Vision_TxView view;
    const Device_Vision_TxResult result =
        Device_Vision_PeekCriticalTx(&g_vision.vision, now_ms, &view);

    if ((result == DEVICE_VISION_TX_SEND_INITIAL) ||
        (result == DEVICE_VISION_TX_SEND_RETRY)) {
        const Device_Serial_Result write_result =
            Device_Serial_WriteBuffered(
                &g_vision.serial, view.bytes, view.length);

        if (write_result == DEVICE_SERIAL_RESULT_BUSY) {
            return true;
        }
        return (write_result == DEVICE_SERIAL_RESULT_OK) &&
            Device_Vision_CommitCriticalTxAccepted(
                &g_vision.vision, now_ms) &&
            Device_Vision_CriticalTxComplete(&g_vision.vision);
    }
    return (result != DEVICE_VISION_TX_FAILED) &&
        (result != DEVICE_VISION_TX_INVALID_ARGUMENT);
}

static void update_snapshots(void)
{
    Device_Vision_ResultSnapshot snapshot;
    Device_Vision_Event event;
    Device_Vision_Stats stats;

    if (Device_Vision_GetLatestResult(&g_vision.vision, &snapshot)) {
        g_vision.status.color = snapshot.result.color;
        g_vision.status.shape = snapshot.result.shape;
        g_vision.status.generation = snapshot.generation;
        g_vision.status.has_result = true;
    }
    while (Device_Vision_ReadCriticalEvent(&g_vision.vision, &event)) {
        g_vision.status.last_event = event;
        g_vision.status.has_event = true;
        if (g_vision.status.event_generation != UINT32_MAX) {
            ++g_vision.status.event_generation;
        }
    }
    if (Device_Vision_GetStats(&g_vision.vision, &stats)) {
        g_vision.status.rejected_frame_count =
            stats.packet_frames_rejected +
            stats.semantic_frames_rejected;
    }
}

bool Task_Hex25E_Vision_Init(void)
{
    const Device_Serial_Config serial_config = {
        .transport_index = HEX25E_CONFIG_VISION_TRANSPORT_INDEX,
        .expected_baud_rate = HEX25E_CONFIG_VISION_BAUD_RATE,
        .minimum_tx_capacity =
            HEX25E_CONFIG_VISION_MIN_TX_CAPACITY,
        .minimum_rx_capacity = 1U,
        .require_buffered_tx = true,
        .require_buffered_rx = true,
    };
    Task_Hex25E_VisionContext next = {0};

    next.vision_config = Device_Vision_DefaultConfig();
    next.vision_config.local_session =
        HEX25E_CONFIG_VISION_LOCAL_SESSION;
    if (!Device_Serial_Init(&next.serial, &serial_config) ||
        !Device_Vision_Init(&next.vision, &next.vision_config)) {
        g_vision = (Task_Hex25E_VisionContext){0};
        return false;
    }
    next.initialized = true;
    g_vision = next;
    return true;
}

bool Task_Hex25E_Vision_Poll(uint32_t now_ms)
{
    uint8_t bytes[HEX25E_CONFIG_VISION_RX_CHUNK_SIZE];
    Device_Serial_RxStatus rx_status;

    if (!g_vision.initialized ||
        !Device_Serial_GetRxStatus(&g_vision.serial, &rx_status)) {
        g_vision.status.fault = true;
        return false;
    }
    if (rx_status.sticky_errors != DEVICE_SERIAL_RX_ERROR_NONE) {
        return recover_link();
    }

    for (size_t chunk = 0U;
         chunk < HEX25E_CONFIG_VISION_MAX_CHUNKS_PER_UPDATE; ++chunk) {
        size_t received = 0U;

        if (Device_Serial_ReadAvailable(&g_vision.serial,
                bytes, sizeof(bytes), &received) !=
            DEVICE_SERIAL_RESULT_OK) {
            g_vision.status.fault = true;
            return false;
        }
        for (size_t index = 0U; index < received; ++index) {
            if (!feed_received_byte(now_ms, bytes[index])) {
                g_vision.status.fault = true;
                return false;
            }
        }
        if (received < sizeof(bytes)) {
            break;
        }
    }
    if (!pump_critical_tx(now_ms)) {
        g_vision.status.fault = true;
        return false;
    }
    update_snapshots();
    g_vision.status.fault = false;
    return true;
}

bool Task_Hex25E_Vision_GetStatus(Task_Hex25E_VisionStatus *status)
{
    if (!g_vision.initialized || (status == NULL)) {
        return false;
    }
    *status = g_vision.status;
    return true;
}

Device_Vision_BeginResult Task_Hex25E_Vision_BeginCommand(
    Device_Vision_Command command, Device_Vision_Request request)
{
    if (!g_vision.initialized) {
        return DEVICE_VISION_BEGIN_INVALID_ARGUMENT;
    }
    return Device_Vision_BeginCommand(
        &g_vision.vision, command, request);
}
