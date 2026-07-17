#ifndef DEVICE_VISION_DEVICE_VISION_H
#define DEVICE_VISION_DEVICE_VISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "algorithm_packet.h"
#include "algorithm_reliable_link.h"

#define DEVICE_VISION_CRITICAL_EVENT_QUEUE_CAPACITY 8U
#define DEVICE_VISION_ACK_FRAME_MAX_SIZE 29U

typedef enum {
    DEVICE_VISION_COLOR_UNKNOWN = 0,
    DEVICE_VISION_COLOR_RED,
    DEVICE_VISION_COLOR_GREEN,
    DEVICE_VISION_COLOR_BLUE,
    DEVICE_VISION_COLOR_YELLOW,
    DEVICE_VISION_COLOR_BLACK,
    DEVICE_VISION_COLOR_WHITE,
} Device_Vision_Color;

typedef enum {
    DEVICE_VISION_SHAPE_UNKNOWN = 0,
    DEVICE_VISION_SHAPE_CIRCLE,
    DEVICE_VISION_SHAPE_TRIANGLE,
    DEVICE_VISION_SHAPE_SQUARE,
    DEVICE_VISION_SHAPE_RECTANGLE,
} Device_Vision_Shape;

typedef struct {
    Device_Vision_Color color;
    Device_Vision_Shape shape;
    uint16_t context_id;
    uint16_t class_id;
    uint16_t confidence_per_mille;
    int16_t center_x;
    int16_t center_y;
    bool has_classification;
} Device_Vision_Result;

typedef enum {
    DEVICE_VISION_REQUEST_COLOR = 0,
    DEVICE_VISION_REQUEST_SHAPE,
    DEVICE_VISION_REQUEST_BOTH,
} Device_Vision_Request;

typedef enum {
    DEVICE_VISION_COMMAND_START = 0,
    DEVICE_VISION_COMMAND_END,
    DEVICE_VISION_COMMAND_REQUEST,
} Device_Vision_Command;

typedef enum {
    DEVICE_VISION_EVENT_REMOTE_START = 0,
    DEVICE_VISION_EVENT_REMOTE_END,
    DEVICE_VISION_EVENT_REMOTE_REQUEST,
    DEVICE_VISION_EVENT_REMOTE_CONTEXT,
} Device_Vision_EventType;

typedef struct {
    Device_Vision_EventType type;
    uint32_t sender_session;
    uint16_t sequence;
    Device_Vision_Request request;
    uint16_t context_id;
} Device_Vision_Event;

typedef struct {
    uint32_t request_sender_session;
    uint16_t request_sequence;
} Device_Vision_Correlation;

typedef struct {
    Device_Vision_Result result;
    Device_Vision_Correlation correlation;
    uint32_t sender_session;
    uint32_t generation;
    uint16_t sequence;
    bool has_correlation;
} Device_Vision_ResultSnapshot;

typedef struct {
    Algorithm_ReliableLink_Config reliable_link;
    /* Required nonzero boot/session epoch. The application must supply a new
     * value after every local boot; DefaultConfig deliberately returns zero. */
    uint32_t local_session;
    uint16_t initial_critical_sequence;
    uint16_t initial_result_sequence;
} Device_Vision_Config;

typedef enum {
    DEVICE_VISION_FEED_IN_PROGRESS = 0,
    DEVICE_VISION_FEED_CRITICAL_EVENT_AND_ACK_READY,
    DEVICE_VISION_FEED_DUPLICATE_ACK_READY,
    DEVICE_VISION_FEED_RESULT_UPDATED,
    DEVICE_VISION_FEED_ACK_ACCEPTED,
    DEVICE_VISION_FEED_ACK_IGNORED,
    DEVICE_VISION_FEED_FRAME_REJECTED,
    DEVICE_VISION_FEED_SEMANTIC_REJECTED,
    DEVICE_VISION_FEED_CRITICAL_QUEUE_FULL,
    DEVICE_VISION_FEED_ACK_OUTPUT_TOO_SMALL,
    DEVICE_VISION_FEED_INVALID_ARGUMENT,
} Device_Vision_FeedResult;

typedef enum {
    DEVICE_VISION_BEGIN_OK = 0,
    DEVICE_VISION_BEGIN_BUSY,
    DEVICE_VISION_BEGIN_INVALID_ARGUMENT,
    DEVICE_VISION_BEGIN_FRAME_ERROR,
} Device_Vision_BeginResult;

typedef enum {
    DEVICE_VISION_TX_IDLE = 0,
    DEVICE_VISION_TX_WAITING_FOR_ACK,
    DEVICE_VISION_TX_IN_FLIGHT,
    DEVICE_VISION_TX_SEND_INITIAL,
    DEVICE_VISION_TX_SEND_RETRY,
    DEVICE_VISION_TX_ACK_LATCHED,
    DEVICE_VISION_TX_FAILED,
    DEVICE_VISION_TX_INVALID_ARGUMENT,
} Device_Vision_TxResult;

typedef struct {
    const uint8_t *bytes;
    size_t length;
    uint16_t sequence;
} Device_Vision_TxView;

typedef struct {
    uint32_t packet_frames_accepted;
    uint32_t packet_frames_rejected;
    uint32_t semantic_frames_rejected;
    uint32_t critical_queue_full_count;
    uint32_t acknowledgements_sent;
    uint32_t acknowledgements_accepted;
    uint32_t acknowledgements_ignored;
    uint32_t duplicate_critical_frames;
    uint32_t result_snapshots_updated;
} Device_Vision_Stats;

/* Caller-owned single-context protocol state. UART ISR/DMA only moves bytes;
 * protocol APIs run in one serialized main/task context. */
typedef struct {
    Algorithm_Packet_Parser parser;
    uint8_t parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    Algorithm_ReliableLink critical_tx;
    Algorithm_ReliableLink_Receiver critical_rx;
    Device_Vision_Event
        critical_events[DEVICE_VISION_CRITICAL_EVENT_QUEUE_CAPACITY];
    Device_Vision_ResultSnapshot latest_result;
    Device_Vision_Stats stats;
    uint32_t local_session;
    uint16_t next_result_sequence;
    uint8_t critical_event_head;
    uint8_t critical_event_tail;
    uint8_t critical_event_count;
    bool has_latest_result;
    bool initialized;
} Device_Vision;

Device_Vision_Config Device_Vision_DefaultConfig(void);
bool Device_Vision_Init(
    Device_Vision *vision, const Device_Vision_Config *config);

/* Feed one received byte and the byte's current monotonic millisecond time.
 * For *_ACK_READY, ack_output contains one complete LF frame. If submitted by
 * zero-copy DMA, the caller must keep that buffer alive and unchanged until
 * the DMA completion callback; do not reuse a stack/scratch buffer in flight.
 * ack_length is zero for every other result. Critical events are not ACKed
 * when their dedicated queue is full. RESULT never consumes that queue. */
Device_Vision_FeedResult Device_Vision_FeedByte(
    Device_Vision *vision,
    uint32_t now_ms,
    uint8_t byte,
    uint8_t *ack_output,
    size_t ack_capacity,
    size_t *ack_length);

Device_Vision_BeginResult Device_Vision_BeginCommand(
    Device_Vision *vision,
    Device_Vision_Command command,
    Device_Vision_Request request);

/* Sends a reliable, application-defined numeric context.  The Device layer
 * does not assign meaning to the value; a Task may use it for a waypoint,
 * workpiece slot, camera pose, or another protocol context. */
Device_Vision_BeginResult Device_Vision_BeginContext(
    Device_Vision *vision, uint16_t context_id);

/* Peek does not consume a send attempt. After UART returns OK, call Commit;
 * after blocking completion or exactly one DMA callback, call TxComplete. */
Device_Vision_TxResult Device_Vision_PeekCriticalTx(
    Device_Vision *vision,
    uint32_t now_ms,
    Device_Vision_TxView *view);
bool Device_Vision_CommitCriticalTxAccepted(
    Device_Vision *vision, uint32_t now_ms);
bool Device_Vision_CriticalTxComplete(Device_Vision *vision);
bool Device_Vision_CriticalTxOutstanding(const Device_Vision *vision);
bool Device_Vision_CriticalTxInFlight(const Device_Vision *vision);

/* A NULL correlation emits an unsolicited/latest telemetry RESULT. A non-null
 * correlation identifies the remote REQUEST event being answered and must
 * contain a nonzero request_sender_session. The caller owns output through any
 * zero-copy DMA completion. */
bool Device_Vision_EncodeResult(
    Device_Vision *vision,
    Device_Vision_Result result,
    const Device_Vision_Correlation *correlation,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

bool Device_Vision_ReadCriticalEvent(
    Device_Vision *vision, Device_Vision_Event *event);
bool Device_Vision_GetLatestResult(
    const Device_Vision *vision, Device_Vision_ResultSnapshot *snapshot);
bool Device_Vision_GetStats(
    const Device_Vision *vision, Device_Vision_Stats *stats);
void Device_Vision_ClearCriticalEvents(Device_Vision *vision);

const char *Device_Vision_ColorName(Device_Vision_Color color);
const char *Device_Vision_ShapeName(Device_Vision_Shape shape);

#endif
