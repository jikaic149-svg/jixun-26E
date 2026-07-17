#include "algorithm_reliable_link.h"

static void clear_send_view(Algorithm_ReliableLink_SendView *view)
{
    if (view != NULL) {
        view->bytes = NULL;
        view->length = 0U;
        view->sequence = 0U;
    }
}

static void fill_send_view(
    const Algorithm_ReliableLink *link,
    Algorithm_ReliableLink_SendView *view)
{
    view->bytes = link->frame;
    view->length = link->frame_length;
    view->sequence = link->outstanding_sequence;
}

static uint16_t increment_sequence(uint16_t sequence)
{
    return (sequence == UINT16_MAX) ? 0U : (uint16_t)(sequence + 1U);
}

static uint32_t retry_due_ms(
    const Algorithm_ReliableLink_Config *config, uint8_t retry_number)
{
    const uint32_t denominator = (uint32_t)config->max_retries + 1U;
    const uint32_t quotient = config->total_timeout_ms / denominator;
    const uint32_t remainder = config->total_timeout_ms % denominator;
    const uint32_t retry = (uint32_t)retry_number;
    const uint32_t scaled_remainder = remainder * retry;
    uint32_t due_ms =
        (quotient * retry) + (scaled_remainder / denominator);

    if ((scaled_remainder % denominator) != 0U) {
        ++due_ms;
    }
    return due_ms;
}

static Algorithm_ReliableLink_BeginResult map_encode_result(
    Algorithm_Packet_EncodeResult result)
{
    switch (result) {
        case ALGORITHM_PACKET_ENCODE_OK:
            return ALGORITHM_RELIABLE_LINK_BEGIN_OK;
        case ALGORITHM_PACKET_ENCODE_INVALID_ARGUMENT:
            return ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_ARGUMENT;
        case ALGORITHM_PACKET_ENCODE_INVALID_TYPE:
            return ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_TYPE;
        case ALGORITHM_PACKET_ENCODE_INVALID_PAYLOAD:
            return ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_PAYLOAD;
        case ALGORITHM_PACKET_ENCODE_FRAME_TOO_LONG:
            return ALGORITHM_RELIABLE_LINK_BEGIN_FRAME_TOO_LONG;
        case ALGORITHM_PACKET_ENCODE_OUTPUT_TOO_SMALL:
        default:
            return ALGORITHM_RELIABLE_LINK_BEGIN_INTERNAL_ERROR;
    }
}

static bool waiting_state(Algorithm_ReliableLink_State state)
{
    return (state == ALGORITHM_RELIABLE_LINK_STATE_WAITING_FOR_ACK) ||
        (state == ALGORITHM_RELIABLE_LINK_STATE_ACK_LATCHED);
}

static bool deadline_reached(
    const Algorithm_ReliableLink *link, uint32_t now_ms)
{
    return ((uint32_t)(now_ms - link->started_at_ms) >=
        link->config.total_timeout_ms);
}

static void finish_acknowledged(Algorithm_ReliableLink *link)
{
    link->frame_length = 0U;
    link->retries_accepted = 0U;
    link->next_retry_due_offset_ms = 0U;
    link->ack_latched = false;
    link->state = ALGORITHM_RELIABLE_LINK_STATE_IDLE;
}

Algorithm_ReliableLink_Config Algorithm_ReliableLink_DefaultConfig(void)
{
    const Algorithm_ReliableLink_Config config = {
        .total_timeout_ms =
            ALGORITHM_RELIABLE_LINK_DEFAULT_TOTAL_TIMEOUT_MS,
        .max_retries = ALGORITHM_RELIABLE_LINK_DEFAULT_MAX_RETRIES,
    };
    return config;
}

bool Algorithm_ReliableLink_Init(
    Algorithm_ReliableLink *link,
    const Algorithm_ReliableLink_Config *config,
    uint16_t initial_sequence)
{
    Algorithm_ReliableLink_Config selected;
    uint8_t retry_number;

    if (link == NULL) {
        return false;
    }
    *link = (Algorithm_ReliableLink){0};
    selected = (config == NULL)
        ? Algorithm_ReliableLink_DefaultConfig()
        : *config;
    if ((selected.total_timeout_ms == 0U) ||
        (selected.max_retries > ALGORITHM_RELIABLE_LINK_MAX_RETRIES) ||
        (selected.total_timeout_ms <= (uint32_t)selected.max_retries)) {
        return false;
    }

    link->config = selected;
    for (retry_number = 1U;
         retry_number <= selected.max_retries;
         ++retry_number) {
        link->retry_due_offsets_ms[retry_number - 1U] =
            retry_due_ms(&selected, retry_number);
    }
    link->next_sequence = initial_sequence;
    link->state = ALGORITHM_RELIABLE_LINK_STATE_IDLE;
    link->initialized = true;
    return true;
}

Algorithm_ReliableLink_BeginResult Algorithm_ReliableLink_Begin(
    Algorithm_ReliableLink *link,
    const char *type,
    const uint8_t *payload,
    size_t payload_length)
{
    Algorithm_Packet_EncodeResult encode_result;
    size_t encoded_length = 0U;
    uint16_t sequence;

    if ((link == NULL) || !link->initialized) {
        return ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_ARGUMENT;
    }
    if (link->tx_in_flight ||
        (link->state == ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND) ||
        waiting_state(link->state)) {
        return ALGORITHM_RELIABLE_LINK_BEGIN_BUSY;
    }

    sequence = link->next_sequence;
    encode_result = Algorithm_Packet_Encode(
        type,
        sequence,
        payload,
        payload_length,
        link->frame,
        sizeof(link->frame),
        &encoded_length);
    if (encode_result != ALGORITHM_PACKET_ENCODE_OK) {
        return map_encode_result(encode_result);
    }

    link->frame_length = encoded_length;
    link->outstanding_sequence = sequence;
    link->next_sequence = increment_sequence(sequence);
    link->started_at_ms = 0U;
    link->next_retry_due_offset_ms = 0U;
    link->retries_accepted = 0U;
    link->ack_latched = false;
    link->state = ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND;
    return ALGORITHM_RELIABLE_LINK_BEGIN_OK;
}

Algorithm_ReliableLink_PeekResult Algorithm_ReliableLink_PeekDueFrame(
    Algorithm_ReliableLink *link,
    uint32_t now_ms,
    Algorithm_ReliableLink_SendView *view)
{
    uint32_t elapsed_ms;

    clear_send_view(view);
    if ((link == NULL) || (view == NULL) || !link->initialized) {
        return ALGORITHM_RELIABLE_LINK_PEEK_INVALID_ARGUMENT;
    }

    switch (link->state) {
        case ALGORITHM_RELIABLE_LINK_STATE_IDLE:
            return ALGORITHM_RELIABLE_LINK_PEEK_IDLE;

        case ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND:
            if (link->tx_in_flight) {
                return ALGORITHM_RELIABLE_LINK_PEEK_TX_IN_FLIGHT;
            }
            fill_send_view(link, view);
            return ALGORITHM_RELIABLE_LINK_PEEK_SEND_INITIAL;

        case ALGORITHM_RELIABLE_LINK_STATE_ACK_LATCHED:
            return ALGORITHM_RELIABLE_LINK_PEEK_ACK_LATCHED;

        case ALGORITHM_RELIABLE_LINK_STATE_WAITING_FOR_ACK:
            if (deadline_reached(link, now_ms)) {
                link->state = ALGORITHM_RELIABLE_LINK_STATE_FAILED;
                return ALGORITHM_RELIABLE_LINK_PEEK_FAILED;
            }
            if (link->tx_in_flight) {
                return ALGORITHM_RELIABLE_LINK_PEEK_TX_IN_FLIGHT;
            }
            if (link->retries_accepted >= link->config.max_retries) {
                return ALGORITHM_RELIABLE_LINK_PEEK_WAITING;
            }
            elapsed_ms = (uint32_t)(now_ms - link->started_at_ms);
            if (elapsed_ms < link->next_retry_due_offset_ms) {
                return ALGORITHM_RELIABLE_LINK_PEEK_WAITING;
            }
            fill_send_view(link, view);
            return ALGORITHM_RELIABLE_LINK_PEEK_SEND_RETRY;

        case ALGORITHM_RELIABLE_LINK_STATE_FAILED:
            return ALGORITHM_RELIABLE_LINK_PEEK_FAILED;

        case ALGORITHM_RELIABLE_LINK_STATE_UNINITIALIZED:
        default:
            return ALGORITHM_RELIABLE_LINK_PEEK_INVALID_ARGUMENT;
    }
}

bool Algorithm_ReliableLink_CommitTxAccepted(
    Algorithm_ReliableLink *link, uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if ((link == NULL) || !link->initialized || link->tx_in_flight) {
        return false;
    }

    if (link->state == ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND) {
        link->started_at_ms = now_ms;
        link->retries_accepted = 0U;
        link->next_retry_due_offset_ms =
            (link->config.max_retries == 0U)
                ? 0U
                : link->retry_due_offsets_ms[0];
        link->tx_in_flight = true;
        link->state = ALGORITHM_RELIABLE_LINK_STATE_WAITING_FOR_ACK;
        return true;
    }

    if (link->state != ALGORITHM_RELIABLE_LINK_STATE_WAITING_FOR_ACK) {
        return false;
    }
    if (deadline_reached(link, now_ms)) {
        link->state = ALGORITHM_RELIABLE_LINK_STATE_FAILED;
        return false;
    }
    if (link->retries_accepted >= link->config.max_retries) {
        return false;
    }
    elapsed_ms = (uint32_t)(now_ms - link->started_at_ms);
    if (elapsed_ms < link->next_retry_due_offset_ms) {
        return false;
    }

    ++link->retries_accepted;
    if (link->retries_accepted < link->config.max_retries) {
        link->next_retry_due_offset_ms =
            link->retry_due_offsets_ms[link->retries_accepted];
    }
    else {
        link->next_retry_due_offset_ms = link->config.total_timeout_ms;
    }
    link->tx_in_flight = true;
    return true;
}

bool Algorithm_ReliableLink_TxComplete(Algorithm_ReliableLink *link)
{
    if ((link == NULL) || !link->initialized || !link->tx_in_flight) {
        return false;
    }

    link->tx_in_flight = false;
    if (link->ack_latched) {
        finish_acknowledged(link);
    }
    return true;
}

bool Algorithm_ReliableLink_Acknowledge(
    Algorithm_ReliableLink *link,
    uint16_t sequence,
    uint32_t now_ms)
{
    if ((link == NULL) || !link->initialized ||
        !waiting_state(link->state) ||
        (sequence != link->outstanding_sequence)) {
        return false;
    }
    if (link->ack_latched) {
        return true;
    }
    if (deadline_reached(link, now_ms)) {
        link->state = ALGORITHM_RELIABLE_LINK_STATE_FAILED;
        return false;
    }

    link->ack_latched = true;
    link->state = ALGORITHM_RELIABLE_LINK_STATE_ACK_LATCHED;
    if (!link->tx_in_flight) {
        finish_acknowledged(link);
    }
    return true;
}

Algorithm_ReliableLink_State Algorithm_ReliableLink_GetState(
    const Algorithm_ReliableLink *link)
{
    return ((link == NULL) || !link->initialized)
        ? ALGORITHM_RELIABLE_LINK_STATE_UNINITIALIZED
        : link->state;
}

bool Algorithm_ReliableLink_HasOutstanding(
    const Algorithm_ReliableLink *link)
{
    if ((link == NULL) || !link->initialized) {
        return false;
    }
    return (link->state == ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND) ||
        waiting_state(link->state);
}

bool Algorithm_ReliableLink_HasTxInFlight(
    const Algorithm_ReliableLink *link)
{
    return (link != NULL) && link->initialized && link->tx_in_flight;
}

bool Algorithm_ReliableLink_GetOutstandingSequence(
    const Algorithm_ReliableLink *link, uint16_t *sequence)
{
    if ((sequence == NULL) || !Algorithm_ReliableLink_HasOutstanding(link)) {
        return false;
    }
    *sequence = link->outstanding_sequence;
    return true;
}

void Algorithm_ReliableLink_Receiver_Init(
    Algorithm_ReliableLink_Receiver *receiver)
{
    if (receiver != NULL) {
        *receiver = (Algorithm_ReliableLink_Receiver){0};
    }
}

Algorithm_ReliableLink_ReceiveDecision
Algorithm_ReliableLink_Receiver_Observe(
    Algorithm_ReliableLink_Receiver *receiver,
    uint32_t session,
    uint16_t sequence)
{
    if ((receiver == NULL) || (session == 0U)) {
        return ALGORITHM_RELIABLE_LINK_RECEIVE_INVALID_ARGUMENT;
    }
    if (receiver->has_last &&
        (receiver->last_session == session) &&
        (receiver->last_sequence == sequence)) {
        return ALGORITHM_RELIABLE_LINK_RECEIVE_DUPLICATE_REACK;
    }

    receiver->last_session = session;
    receiver->last_sequence = sequence;
    receiver->has_last = true;
    return ALGORITHM_RELIABLE_LINK_RECEIVE_DELIVER_AND_ACK;
}
