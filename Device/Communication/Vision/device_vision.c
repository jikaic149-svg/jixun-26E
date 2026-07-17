#include "device_vision.h"

#include <limits.h>
#include <string.h>

#define DEVICE_VISION_RESULT_PAYLOAD_MAX_SIZE 40U

static void Device_Vision_IncrementSaturating(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

static uint32_t Device_Vision_NextGeneration(uint32_t generation)
{
    return (generation == UINT32_MAX) ? 1U : generation + 1U;
}

static uint16_t Device_Vision_IncrementSequence(uint16_t sequence)
{
    return (sequence == UINT16_MAX) ? 0U : (uint16_t)(sequence + 1U);
}

static bool Device_Vision_SliceEquals(
    const uint8_t *slice, size_t length, const char *text)
{
    const size_t text_length = strlen(text);

    return (length == text_length) &&
        (memcmp(slice, text, text_length) == 0);
}

static bool Device_Vision_TypeEquals(
    const Algorithm_Packet_View *packet, const char *type)
{
    return Device_Vision_SliceEquals(
        packet->type, packet->type_length, type);
}

static uint8_t Device_Vision_HexDigit(uint8_t nibble)
{
    static const uint8_t digits[] = "0123456789ABCDEF";

    return digits[nibble & 0x0FU];
}

static void Device_Vision_WriteHex32(uint32_t value, uint8_t output[8U])
{
    unsigned int index;

    for (index = 0U; index < 8U; ++index) {
        const unsigned int shift = (7U - index) * 4U;
        output[index] = Device_Vision_HexDigit(
            (uint8_t)(value >> shift));
    }
}

static void Device_Vision_WriteHex16(uint16_t value, uint8_t output[4U])
{
    unsigned int index;

    for (index = 0U; index < 4U; ++index) {
        const unsigned int shift = (3U - index) * 4U;
        output[index] = Device_Vision_HexDigit(
            (uint8_t)(value >> shift));
    }
}

static bool Device_Vision_ParseHex(
    const uint8_t *bytes, size_t length, uint32_t *value)
{
    uint32_t parsed = 0U;
    size_t index;

    if ((bytes == NULL) || (value == NULL) ||
        ((length != 4U) && (length != 8U))) {
        return false;
    }
    for (index = 0U; index < length; ++index) {
        uint32_t nibble;

        if ((bytes[index] >= (uint8_t)'0') &&
            (bytes[index] <= (uint8_t)'9')) {
            nibble = (uint32_t)(bytes[index] - (uint8_t)'0');
        }
        else if ((bytes[index] >= (uint8_t)'A') &&
            (bytes[index] <= (uint8_t)'F')) {
            nibble = (uint32_t)(bytes[index] - (uint8_t)'A') + 10U;
        }
        else {
            return false;
        }
        parsed = (parsed << 4U) | nibble;
    }
    *value = parsed;
    return true;
}

static bool Device_Vision_ParseUnsignedDecimal(
    const uint8_t *bytes,
    size_t length,
    uint32_t maximum,
    uint32_t *value)
{
    uint32_t parsed = 0U;
    size_t index;

    if ((bytes == NULL) || (value == NULL) || (length == 0U) ||
        ((length > 1U) && (bytes[0] == (uint8_t)'0'))) {
        return false;
    }
    for (index = 0U; index < length; ++index) {
        uint32_t digit;

        if ((bytes[index] < (uint8_t)'0') ||
            (bytes[index] > (uint8_t)'9')) {
            return false;
        }
        digit = (uint32_t)(bytes[index] - (uint8_t)'0');
        if ((parsed > (maximum / 10U)) ||
            ((parsed == (maximum / 10U)) &&
             (digit > (maximum % 10U)))) {
            return false;
        }
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return true;
}

static size_t Device_Vision_WriteUnsignedDecimal(
    uint16_t value, uint8_t *output)
{
    uint8_t reversed[5U];
    size_t length = 0U;
    size_t index;

    do {
        reversed[length++] = (uint8_t)('0' + (value % 10U));
        value = (uint16_t)(value / 10U);
    } while (value != 0U);
    for (index = 0U; index < length; ++index) {
        output[index] = reversed[length - index - 1U];
    }
    return length;
}

static bool Device_Vision_ParseSession(
    const uint8_t *bytes, size_t length, uint32_t *session)
{
    return Device_Vision_ParseHex(bytes, length, session) &&
        (*session != 0U);
}

static bool Device_Vision_QueueCriticalEvent(
    Device_Vision *vision, const Device_Vision_Event *event)
{
    if (vision->critical_event_count >=
        DEVICE_VISION_CRITICAL_EVENT_QUEUE_CAPACITY) {
        Device_Vision_IncrementSaturating(
            &vision->stats.critical_queue_full_count);
        return false;
    }

    vision->critical_events[vision->critical_event_tail] = *event;
    vision->critical_event_tail = (uint8_t)(
        (vision->critical_event_tail + 1U) %
            DEVICE_VISION_CRITICAL_EVENT_QUEUE_CAPACITY);
    ++vision->critical_event_count;
    return true;
}

static bool Device_Vision_ParseRequestName(
    const uint8_t *payload,
    size_t length,
    Device_Vision_Request *request)
{
    if (Device_Vision_SliceEquals(payload, length, "COLOR")) {
        *request = DEVICE_VISION_REQUEST_COLOR;
        return true;
    }
    if (Device_Vision_SliceEquals(payload, length, "SHAPE")) {
        *request = DEVICE_VISION_REQUEST_SHAPE;
        return true;
    }
    if (Device_Vision_SliceEquals(payload, length, "BOTH")) {
        *request = DEVICE_VISION_REQUEST_BOTH;
        return true;
    }
    return false;
}

static bool Device_Vision_ParseColor(
    const uint8_t *payload,
    size_t length,
    Device_Vision_Color *color)
{
    static const struct {
        const char *name;
        Device_Vision_Color value;
    } values[] = {
        {"UNKNOWN", DEVICE_VISION_COLOR_UNKNOWN},
        {"RED", DEVICE_VISION_COLOR_RED},
        {"GREEN", DEVICE_VISION_COLOR_GREEN},
        {"BLUE", DEVICE_VISION_COLOR_BLUE},
        {"YELLOW", DEVICE_VISION_COLOR_YELLOW},
        {"BLACK", DEVICE_VISION_COLOR_BLACK},
        {"WHITE", DEVICE_VISION_COLOR_WHITE},
    };
    size_t index;

    for (index = 0U; index < (sizeof(values) / sizeof(values[0])); ++index) {
        if (Device_Vision_SliceEquals(payload, length, values[index].name)) {
            *color = values[index].value;
            return true;
        }
    }
    return false;
}

static bool Device_Vision_ParseShape(
    const uint8_t *payload,
    size_t length,
    Device_Vision_Shape *shape)
{
    static const struct {
        const char *name;
        Device_Vision_Shape value;
    } values[] = {
        {"UNKNOWN", DEVICE_VISION_SHAPE_UNKNOWN},
        {"CIRCLE", DEVICE_VISION_SHAPE_CIRCLE},
        {"TRIANGLE", DEVICE_VISION_SHAPE_TRIANGLE},
        {"SQUARE", DEVICE_VISION_SHAPE_SQUARE},
        {"RECTANGLE", DEVICE_VISION_SHAPE_RECTANGLE},
    };
    size_t index;

    for (index = 0U; index < (sizeof(values) / sizeof(values[0])); ++index) {
        if (Device_Vision_SliceEquals(payload, length, values[index].name)) {
            *shape = values[index].value;
            return true;
        }
    }
    return false;
}

static bool Device_Vision_SplitFields(
    const uint8_t *payload,
    size_t length,
    const uint8_t **fields,
    size_t *field_lengths,
    size_t maximum_fields,
    size_t *field_count)
{
    size_t start = 0U;
    size_t count = 0U;
    size_t index;

    if ((payload == NULL) || (fields == NULL) ||
        (field_lengths == NULL) || (field_count == NULL) ||
        (maximum_fields == 0U)) {
        return false;
    }
    for (index = 0U; index <= length; ++index) {
        if ((index == length) || (payload[index] == (uint8_t)',')) {
            if (count >= maximum_fields) {
                return false;
            }
            fields[count] = &payload[start];
            field_lengths[count] = index - start;
            ++count;
            start = index + 1U;
        }
    }
    *field_count = count;
    return true;
}

static bool Device_Vision_ParseResult(
    const uint8_t *payload,
    size_t length,
    Device_Vision_ResultSnapshot *snapshot)
{
    const uint8_t *fields[5U];
    size_t field_lengths[5U];
    size_t field_count;
    uint32_t request_sequence;

    if (!Device_Vision_SplitFields(payload, length,
            fields, field_lengths, 5U, &field_count) ||
        ((field_count != 4U) && (field_count != 5U)) ||
        !Device_Vision_ParseSession(
            fields[0], field_lengths[0], &snapshot->sender_session)) {
        return false;
    }

    snapshot->has_correlation = false;
    snapshot->result.has_classification = false;
    snapshot->result.context_id = 0U;
    snapshot->result.class_id = 0U;
    snapshot->result.confidence_per_mille = 0U;
    snapshot->result.center_x = 0;
    snapshot->result.center_y = 0;
    snapshot->correlation.request_sender_session = 0U;
    snapshot->correlation.request_sequence = 0U;
    if (field_count == 4U) {
        if (!Device_Vision_SliceEquals(
                fields[1], field_lengths[1], "NONE") ||
            !Device_Vision_ParseColor(
                fields[2], field_lengths[2], &snapshot->result.color) ||
            !Device_Vision_ParseShape(
                fields[3], field_lengths[3], &snapshot->result.shape)) {
            return false;
        }
        return true;
    }

    if (!Device_Vision_ParseSession(fields[1], field_lengths[1],
            &snapshot->correlation.request_sender_session) ||
        !Device_Vision_ParseHex(
            fields[2], field_lengths[2], &request_sequence) ||
        !Device_Vision_ParseColor(
            fields[3], field_lengths[3], &snapshot->result.color) ||
        !Device_Vision_ParseShape(
            fields[4], field_lengths[4], &snapshot->result.shape)) {
        return false;
    }
    snapshot->correlation.request_sequence = (uint16_t)request_sequence;
    snapshot->has_correlation = true;
    return true;
}

static bool Device_Vision_ParseDetection(
    const uint8_t *payload,
    size_t length,
    Device_Vision_ResultSnapshot *snapshot)
{
    const uint8_t *fields[7U];
    size_t field_lengths[7U];
    size_t field_count;
    uint32_t context_id;
    uint32_t class_id;
    uint32_t confidence;
    uint32_t center_x;
    uint32_t center_y;

    if (!Device_Vision_SplitFields(payload, length,
            fields, field_lengths, 7U, &field_count) ||
        (field_count != 7U) ||
        !Device_Vision_ParseSession(
            fields[0], field_lengths[0], &snapshot->sender_session) ||
        !Device_Vision_ParseUnsignedDecimal(
            fields[1], field_lengths[1], UINT16_MAX, &context_id) ||
        !Device_Vision_ParseUnsignedDecimal(
            fields[2], field_lengths[2], UINT16_MAX, &class_id) ||
        !Device_Vision_ParseShape(
            fields[3], field_lengths[3], &snapshot->result.shape) ||
        !Device_Vision_ParseUnsignedDecimal(
            fields[4], field_lengths[4], 1000U, &confidence) ||
        !Device_Vision_ParseUnsignedDecimal(
            fields[5], field_lengths[5], INT16_MAX, &center_x) ||
        !Device_Vision_ParseUnsignedDecimal(
            fields[6], field_lengths[6], INT16_MAX, &center_y)) {
        return false;
    }

    snapshot->result.color = DEVICE_VISION_COLOR_UNKNOWN;
    snapshot->result.context_id = (uint16_t)context_id;
    snapshot->result.class_id = (uint16_t)class_id;
    snapshot->result.confidence_per_mille = (uint16_t)confidence;
    snapshot->result.center_x = (int16_t)center_x;
    snapshot->result.center_y = (int16_t)center_y;
    snapshot->result.has_classification = true;
    snapshot->has_correlation = false;
    snapshot->correlation.request_sender_session = 0U;
    snapshot->correlation.request_sequence = 0U;
    return true;
}

static Device_Vision_FeedResult Device_Vision_EncodeAck(
    Device_Vision *vision,
    uint32_t confirmed_session,
    uint16_t sequence,
    uint8_t *output,
    size_t capacity,
    size_t *length,
    Device_Vision_FeedResult success_result)
{
    uint8_t payload[8U];
    Algorithm_Packet_EncodeResult result;

    Device_Vision_WriteHex32(confirmed_session, payload);
    result = Algorithm_Packet_Encode(
        "ACK", sequence, payload, sizeof(payload), output, capacity, length);
    if (result != ALGORITHM_PACKET_ENCODE_OK) {
        *length = 0U;
        return DEVICE_VISION_FEED_ACK_OUTPUT_TOO_SMALL;
    }
    Device_Vision_IncrementSaturating(
        &vision->stats.acknowledgements_sent);
    return success_result;
}

static Device_Vision_FeedResult Device_Vision_HandleCritical(
    Device_Vision *vision,
    const Algorithm_Packet_View *packet,
    const Device_Vision_Event *event,
    uint8_t *ack_output,
    size_t ack_capacity,
    size_t *ack_length)
{
    Algorithm_ReliableLink_Receiver next_receiver = vision->critical_rx;
    const Algorithm_ReliableLink_ReceiveDecision decision =
        Algorithm_ReliableLink_Receiver_Observe(
            &next_receiver, event->sender_session, packet->sequence);

    if (decision == ALGORITHM_RELIABLE_LINK_RECEIVE_INVALID_ARGUMENT) {
        return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
    }
    if (decision == ALGORITHM_RELIABLE_LINK_RECEIVE_DUPLICATE_REACK) {
        Device_Vision_IncrementSaturating(
            &vision->stats.duplicate_critical_frames);
        return Device_Vision_EncodeAck(
            vision,
            event->sender_session,
            packet->sequence,
            ack_output,
            ack_capacity,
            ack_length,
            DEVICE_VISION_FEED_DUPLICATE_ACK_READY);
    }

    if (!Device_Vision_QueueCriticalEvent(vision, event)) {
        return DEVICE_VISION_FEED_CRITICAL_QUEUE_FULL;
    }
    vision->critical_rx = next_receiver;
    return Device_Vision_EncodeAck(
        vision,
        event->sender_session,
        packet->sequence,
        ack_output,
        ack_capacity,
        ack_length,
        DEVICE_VISION_FEED_CRITICAL_EVENT_AND_ACK_READY);
}

static bool Device_Vision_ParseCriticalSession(
    const Algorithm_Packet_View *packet, uint32_t *session)
{
    return Device_Vision_ParseSession(
        packet->payload, packet->payload_length, session);
}

static Device_Vision_FeedResult Device_Vision_HandlePacket(
    Device_Vision *vision,
    uint32_t now_ms,
    const Algorithm_Packet_View *packet,
    uint8_t *ack_output,
    size_t ack_capacity,
    size_t *ack_length)
{
    Device_Vision_Event event = {0};
    Device_Vision_ResultSnapshot snapshot = {0};
    uint32_t ack_session;

    if (Device_Vision_TypeEquals(packet, "ACK")) {
        if (!Device_Vision_ParseSession(
                packet->payload, packet->payload_length, &ack_session)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        if ((ack_session == vision->local_session) &&
            Algorithm_ReliableLink_Acknowledge(
                &vision->critical_tx, packet->sequence, now_ms)) {
            Device_Vision_IncrementSaturating(
                &vision->stats.acknowledgements_accepted);
            return DEVICE_VISION_FEED_ACK_ACCEPTED;
        }
        Device_Vision_IncrementSaturating(
            &vision->stats.acknowledgements_ignored);
        return DEVICE_VISION_FEED_ACK_IGNORED;
    }

    event.sequence = packet->sequence;
    if (Device_Vision_TypeEquals(packet, "START")) {
        if (!Device_Vision_ParseCriticalSession(
                packet, &event.sender_session)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        event.type = DEVICE_VISION_EVENT_REMOTE_START;
        return Device_Vision_HandleCritical(
            vision, packet, &event, ack_output, ack_capacity, ack_length);
    }
    if (Device_Vision_TypeEquals(packet, "END")) {
        if (!Device_Vision_ParseCriticalSession(
                packet, &event.sender_session)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        event.type = DEVICE_VISION_EVENT_REMOTE_END;
        return Device_Vision_HandleCritical(
            vision, packet, &event, ack_output, ack_capacity, ack_length);
    }
    if (Device_Vision_TypeEquals(packet, "REQUEST")) {
        if ((packet->payload_length < 10U) ||
            (packet->payload[8] != (uint8_t)',') ||
            !Device_Vision_ParseSession(
                packet->payload, 8U, &event.sender_session) ||
            !Device_Vision_ParseRequestName(
                &packet->payload[9], packet->payload_length - 9U,
                &event.request)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        event.type = DEVICE_VISION_EVENT_REMOTE_REQUEST;
        return Device_Vision_HandleCritical(
            vision, packet, &event, ack_output, ack_capacity, ack_length);
    }
    if (Device_Vision_TypeEquals(packet, "CONTEXT")) {
        uint32_t context_id;

        if ((packet->payload_length < 10U) ||
            (packet->payload[8] != (uint8_t)',') ||
            !Device_Vision_ParseSession(
                packet->payload, 8U, &event.sender_session) ||
            !Device_Vision_ParseUnsignedDecimal(
                &packet->payload[9], packet->payload_length - 9U,
                UINT16_MAX, &context_id)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        event.type = DEVICE_VISION_EVENT_REMOTE_CONTEXT;
        event.context_id = (uint16_t)context_id;
        return Device_Vision_HandleCritical(
            vision, packet, &event, ack_output, ack_capacity, ack_length);
    }
    if (Device_Vision_TypeEquals(packet, "RESULT")) {
        if (!Device_Vision_ParseResult(
                packet->payload, packet->payload_length, &snapshot)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        snapshot.sequence = packet->sequence;
        snapshot.generation = Device_Vision_NextGeneration(
            vision->has_latest_result
                ? vision->latest_result.generation
                : 0U);
        vision->latest_result = snapshot;
        vision->has_latest_result = true;
        Device_Vision_IncrementSaturating(
            &vision->stats.result_snapshots_updated);
        return DEVICE_VISION_FEED_RESULT_UPDATED;
    }
    if (Device_Vision_TypeEquals(packet, "DETECTION")) {
        if (!Device_Vision_ParseDetection(
                packet->payload, packet->payload_length, &snapshot)) {
            return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
        }
        snapshot.sequence = packet->sequence;
        snapshot.generation = Device_Vision_NextGeneration(
            vision->has_latest_result
                ? vision->latest_result.generation
                : 0U);
        vision->latest_result = snapshot;
        vision->has_latest_result = true;
        Device_Vision_IncrementSaturating(
            &vision->stats.result_snapshots_updated);
        return DEVICE_VISION_FEED_RESULT_UPDATED;
    }

    return DEVICE_VISION_FEED_SEMANTIC_REJECTED;
}

Device_Vision_Config Device_Vision_DefaultConfig(void)
{
    Device_Vision_Config config;

    config.reliable_link = Algorithm_ReliableLink_DefaultConfig();
    config.local_session = 0U;
    config.initial_critical_sequence = 0U;
    config.initial_result_sequence = 0U;
    return config;
}

bool Device_Vision_Init(
    Device_Vision *vision, const Device_Vision_Config *config)
{
    Device_Vision_Config selected;

    if (vision == NULL) {
        return false;
    }
    *vision = (Device_Vision){0};
    selected = (config == NULL) ? Device_Vision_DefaultConfig() : *config;
    if ((selected.local_session == 0U) ||
        !Algorithm_Packet_Parser_Init(
            &vision->parser,
            vision->parser_storage,
            sizeof(vision->parser_storage)) ||
        !Algorithm_ReliableLink_Init(
            &vision->critical_tx,
            &selected.reliable_link,
            selected.initial_critical_sequence)) {
        return false;
    }

    Algorithm_ReliableLink_Receiver_Init(&vision->critical_rx);
    vision->local_session = selected.local_session;
    vision->next_result_sequence = selected.initial_result_sequence;
    vision->latest_result.result.color = DEVICE_VISION_COLOR_UNKNOWN;
    vision->latest_result.result.shape = DEVICE_VISION_SHAPE_UNKNOWN;
    vision->initialized = true;
    return true;
}

Device_Vision_FeedResult Device_Vision_FeedByte(
    Device_Vision *vision,
    uint32_t now_ms,
    uint8_t byte,
    uint8_t *ack_output,
    size_t ack_capacity,
    size_t *ack_length)
{
    Algorithm_Packet_View packet;
    Algorithm_Packet_FeedResult result;
    Device_Vision_FeedResult semantic_result;

    if (ack_length != NULL) {
        *ack_length = 0U;
    }
    if ((vision == NULL) || !vision->initialized ||
        (ack_output == NULL) || (ack_length == NULL)) {
        return DEVICE_VISION_FEED_INVALID_ARGUMENT;
    }

    result = Algorithm_Packet_Parser_Feed(&vision->parser, byte, &packet);
    if (result == ALGORITHM_PACKET_FEED_IN_PROGRESS) {
        return DEVICE_VISION_FEED_IN_PROGRESS;
    }
    if (result == ALGORITHM_PACKET_FEED_FRAME_REJECTED) {
        vision->stats.packet_frames_rejected =
            Algorithm_Packet_Parser_ErrorCount(&vision->parser);
        return DEVICE_VISION_FEED_FRAME_REJECTED;
    }
    if (result != ALGORITHM_PACKET_FEED_FRAME_READY) {
        return DEVICE_VISION_FEED_INVALID_ARGUMENT;
    }

    vision->stats.packet_frames_accepted =
        Algorithm_Packet_Parser_AcceptedCount(&vision->parser);
    semantic_result = Device_Vision_HandlePacket(
        vision,
        now_ms,
        &packet,
        ack_output,
        ack_capacity,
        ack_length);
    if (semantic_result == DEVICE_VISION_FEED_SEMANTIC_REJECTED) {
        Device_Vision_IncrementSaturating(
            &vision->stats.semantic_frames_rejected);
    }
    return semantic_result;
}

Device_Vision_BeginResult Device_Vision_BeginCommand(
    Device_Vision *vision,
    Device_Vision_Command command,
    Device_Vision_Request request)
{
    const char *type;
    const char *request_name = NULL;
    uint8_t payload[14U];
    size_t payload_length = 8U;
    Algorithm_ReliableLink_BeginResult result;

    if ((vision == NULL) || !vision->initialized) {
        return DEVICE_VISION_BEGIN_INVALID_ARGUMENT;
    }

    switch (command) {
        case DEVICE_VISION_COMMAND_START:
            type = "START";
            break;
        case DEVICE_VISION_COMMAND_END:
            type = "END";
            break;
        case DEVICE_VISION_COMMAND_REQUEST:
            type = "REQUEST";
            switch (request) {
                case DEVICE_VISION_REQUEST_COLOR:
                    request_name = "COLOR";
                    break;
                case DEVICE_VISION_REQUEST_SHAPE:
                    request_name = "SHAPE";
                    break;
                case DEVICE_VISION_REQUEST_BOTH:
                    request_name = "BOTH";
                    break;
                default:
                    return DEVICE_VISION_BEGIN_INVALID_ARGUMENT;
            }
            break;
        default:
            return DEVICE_VISION_BEGIN_INVALID_ARGUMENT;
    }

    Device_Vision_WriteHex32(vision->local_session, payload);
    if (request_name != NULL) {
        const size_t request_length = strlen(request_name);

        payload[8] = (uint8_t)',';
        memcpy(&payload[9], request_name, request_length);
        payload_length = 9U + request_length;
    }
    result = Algorithm_ReliableLink_Begin(
        &vision->critical_tx, type, payload, payload_length);
    if (result == ALGORITHM_RELIABLE_LINK_BEGIN_OK) {
        return DEVICE_VISION_BEGIN_OK;
    }
    if (result == ALGORITHM_RELIABLE_LINK_BEGIN_BUSY) {
        return DEVICE_VISION_BEGIN_BUSY;
    }
    return DEVICE_VISION_BEGIN_FRAME_ERROR;
}

Device_Vision_BeginResult Device_Vision_BeginContext(
    Device_Vision *vision, uint16_t context_id)
{
    uint8_t payload[14U];
    size_t payload_length = 9U;
    Algorithm_ReliableLink_BeginResult result;

    if ((vision == NULL) || !vision->initialized) {
        return DEVICE_VISION_BEGIN_INVALID_ARGUMENT;
    }
    Device_Vision_WriteHex32(vision->local_session, payload);
    payload[8] = (uint8_t)',';
    payload_length += Device_Vision_WriteUnsignedDecimal(
        context_id, &payload[payload_length]);
    result = Algorithm_ReliableLink_Begin(
        &vision->critical_tx, "CONTEXT", payload, payload_length);
    if (result == ALGORITHM_RELIABLE_LINK_BEGIN_OK) {
        return DEVICE_VISION_BEGIN_OK;
    }
    if (result == ALGORITHM_RELIABLE_LINK_BEGIN_BUSY) {
        return DEVICE_VISION_BEGIN_BUSY;
    }
    return DEVICE_VISION_BEGIN_FRAME_ERROR;
}

Device_Vision_TxResult Device_Vision_PeekCriticalTx(
    Device_Vision *vision,
    uint32_t now_ms,
    Device_Vision_TxView *view)
{
    Algorithm_ReliableLink_SendView algorithm_view;
    Algorithm_ReliableLink_PeekResult result;

    if (view != NULL) {
        view->bytes = NULL;
        view->length = 0U;
        view->sequence = 0U;
    }
    if ((vision == NULL) || !vision->initialized || (view == NULL)) {
        return DEVICE_VISION_TX_INVALID_ARGUMENT;
    }

    result = Algorithm_ReliableLink_PeekDueFrame(
        &vision->critical_tx, now_ms, &algorithm_view);
    if ((result == ALGORITHM_RELIABLE_LINK_PEEK_SEND_INITIAL) ||
        (result == ALGORITHM_RELIABLE_LINK_PEEK_SEND_RETRY)) {
        view->bytes = algorithm_view.bytes;
        view->length = algorithm_view.length;
        view->sequence = algorithm_view.sequence;
        return (result == ALGORITHM_RELIABLE_LINK_PEEK_SEND_INITIAL)
            ? DEVICE_VISION_TX_SEND_INITIAL
            : DEVICE_VISION_TX_SEND_RETRY;
    }
    switch (result) {
        case ALGORITHM_RELIABLE_LINK_PEEK_IDLE:
            return DEVICE_VISION_TX_IDLE;
        case ALGORITHM_RELIABLE_LINK_PEEK_WAITING:
            return DEVICE_VISION_TX_WAITING_FOR_ACK;
        case ALGORITHM_RELIABLE_LINK_PEEK_TX_IN_FLIGHT:
            return DEVICE_VISION_TX_IN_FLIGHT;
        case ALGORITHM_RELIABLE_LINK_PEEK_ACK_LATCHED:
            return DEVICE_VISION_TX_ACK_LATCHED;
        case ALGORITHM_RELIABLE_LINK_PEEK_FAILED:
            return DEVICE_VISION_TX_FAILED;
        default:
            return DEVICE_VISION_TX_INVALID_ARGUMENT;
    }
}

bool Device_Vision_CommitCriticalTxAccepted(
    Device_Vision *vision, uint32_t now_ms)
{
    return (vision != NULL) && vision->initialized &&
        Algorithm_ReliableLink_CommitTxAccepted(
            &vision->critical_tx, now_ms);
}

bool Device_Vision_CriticalTxComplete(Device_Vision *vision)
{
    return (vision != NULL) && vision->initialized &&
        Algorithm_ReliableLink_TxComplete(&vision->critical_tx);
}

bool Device_Vision_CriticalTxOutstanding(const Device_Vision *vision)
{
    return (vision != NULL) && vision->initialized &&
        Algorithm_ReliableLink_HasOutstanding(&vision->critical_tx);
}

bool Device_Vision_CriticalTxInFlight(const Device_Vision *vision)
{
    return (vision != NULL) && vision->initialized &&
        Algorithm_ReliableLink_HasTxInFlight(&vision->critical_tx);
}

bool Device_Vision_EncodeResult(
    Device_Vision *vision,
    Device_Vision_Result result,
    const Device_Vision_Correlation *correlation,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    const char *color;
    const char *shape;
    uint8_t payload[DEVICE_VISION_RESULT_PAYLOAD_MAX_SIZE];
    size_t position = 0U;
    size_t color_length;
    size_t shape_length;
    Algorithm_Packet_EncodeResult encode_result;

    if (output_length != NULL) {
        *output_length = 0U;
    }
    if ((vision == NULL) || !vision->initialized || (output == NULL) ||
        (output_length == NULL) ||
        ((unsigned int)result.color >
         (unsigned int)DEVICE_VISION_COLOR_WHITE) ||
        ((unsigned int)result.shape >
         (unsigned int)DEVICE_VISION_SHAPE_RECTANGLE) ||
        ((correlation != NULL) &&
         (correlation->request_sender_session == 0U))) {
        return false;
    }

    color = Device_Vision_ColorName(result.color);
    shape = Device_Vision_ShapeName(result.shape);
    color_length = strlen(color);
    shape_length = strlen(shape);
    Device_Vision_WriteHex32(vision->local_session, &payload[position]);
    position += 8U;
    payload[position++] = (uint8_t)',';
    if (correlation == NULL) {
        memcpy(&payload[position], "NONE", 4U);
        position += 4U;
    }
    else {
        Device_Vision_WriteHex32(
            correlation->request_sender_session, &payload[position]);
        position += 8U;
        payload[position++] = (uint8_t)',';
        Device_Vision_WriteHex16(
            correlation->request_sequence, &payload[position]);
        position += 4U;
    }
    payload[position++] = (uint8_t)',';
    memcpy(&payload[position], color, color_length);
    position += color_length;
    payload[position++] = (uint8_t)',';
    memcpy(&payload[position], shape, shape_length);
    position += shape_length;

    encode_result = Algorithm_Packet_Encode(
        "RESULT",
        vision->next_result_sequence,
        payload,
        position,
        output,
        output_capacity,
        output_length);
    if (encode_result != ALGORITHM_PACKET_ENCODE_OK) {
        return false;
    }
    vision->next_result_sequence =
        Device_Vision_IncrementSequence(vision->next_result_sequence);
    return true;
}

bool Device_Vision_ReadCriticalEvent(
    Device_Vision *vision, Device_Vision_Event *event)
{
    if ((vision == NULL) || !vision->initialized || (event == NULL) ||
        (vision->critical_event_count == 0U)) {
        return false;
    }

    *event = vision->critical_events[vision->critical_event_head];
    vision->critical_event_head = (uint8_t)(
        (vision->critical_event_head + 1U) %
            DEVICE_VISION_CRITICAL_EVENT_QUEUE_CAPACITY);
    --vision->critical_event_count;
    return true;
}

bool Device_Vision_GetLatestResult(
    const Device_Vision *vision, Device_Vision_ResultSnapshot *snapshot)
{
    if ((vision == NULL) || !vision->initialized || (snapshot == NULL) ||
        !vision->has_latest_result) {
        return false;
    }
    *snapshot = vision->latest_result;
    return true;
}

bool Device_Vision_GetStats(
    const Device_Vision *vision, Device_Vision_Stats *stats)
{
    if ((vision == NULL) || !vision->initialized || (stats == NULL)) {
        return false;
    }
    *stats = vision->stats;
    return true;
}

void Device_Vision_ClearCriticalEvents(Device_Vision *vision)
{
    if ((vision == NULL) || !vision->initialized) {
        return;
    }
    vision->critical_event_head = 0U;
    vision->critical_event_tail = 0U;
    vision->critical_event_count = 0U;
}

const char *Device_Vision_ColorName(Device_Vision_Color color)
{
    static const char *const names[] = {
        "UNKNOWN", "RED", "GREEN", "BLUE", "YELLOW", "BLACK", "WHITE",
    };

    return ((unsigned int)color <=
            (unsigned int)DEVICE_VISION_COLOR_WHITE)
        ? names[color]
        : "UNKNOWN";
}

const char *Device_Vision_ShapeName(Device_Vision_Shape shape)
{
    static const char *const names[] = {
        "UNKNOWN", "CIRCLE", "TRIANGLE", "SQUARE", "RECTANGLE",
    };

    return ((unsigned int)shape <=
            (unsigned int)DEVICE_VISION_SHAPE_RECTANGLE)
        ? names[shape]
        : "UNKNOWN";
}
