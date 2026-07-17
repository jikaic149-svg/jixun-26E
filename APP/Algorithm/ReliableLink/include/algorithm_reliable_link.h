#ifndef ALGORITHM_RELIABLE_LINK_H
#define ALGORITHM_RELIABLE_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "algorithm_packet.h"

#define ALGORITHM_RELIABLE_LINK_DEFAULT_TOTAL_TIMEOUT_MS 30U
#define ALGORITHM_RELIABLE_LINK_DEFAULT_MAX_RETRIES 3U
#define ALGORITHM_RELIABLE_LINK_MAX_RETRIES 8U

typedef struct {
    uint32_t total_timeout_ms;
    uint8_t max_retries;
} Algorithm_ReliableLink_Config;

typedef enum {
    ALGORITHM_RELIABLE_LINK_STATE_UNINITIALIZED = 0,
    ALGORITHM_RELIABLE_LINK_STATE_IDLE,
    ALGORITHM_RELIABLE_LINK_STATE_READY_TO_SEND,
    ALGORITHM_RELIABLE_LINK_STATE_WAITING_FOR_ACK,
    ALGORITHM_RELIABLE_LINK_STATE_ACK_LATCHED,
    ALGORITHM_RELIABLE_LINK_STATE_FAILED
} Algorithm_ReliableLink_State;

typedef enum {
    ALGORITHM_RELIABLE_LINK_BEGIN_OK = 0,
    ALGORITHM_RELIABLE_LINK_BEGIN_BUSY,
    ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_ARGUMENT,
    ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_TYPE,
    ALGORITHM_RELIABLE_LINK_BEGIN_INVALID_PAYLOAD,
    ALGORITHM_RELIABLE_LINK_BEGIN_FRAME_TOO_LONG,
    ALGORITHM_RELIABLE_LINK_BEGIN_INTERNAL_ERROR
} Algorithm_ReliableLink_BeginResult;

typedef struct {
    const uint8_t *bytes;
    size_t length;
    uint16_t sequence;
} Algorithm_ReliableLink_SendView;

typedef enum {
    ALGORITHM_RELIABLE_LINK_PEEK_IDLE = 0,
    ALGORITHM_RELIABLE_LINK_PEEK_WAITING,
    ALGORITHM_RELIABLE_LINK_PEEK_TX_IN_FLIGHT,
    ALGORITHM_RELIABLE_LINK_PEEK_SEND_INITIAL,
    ALGORITHM_RELIABLE_LINK_PEEK_SEND_RETRY,
    ALGORITHM_RELIABLE_LINK_PEEK_ACK_LATCHED,
    ALGORITHM_RELIABLE_LINK_PEEK_FAILED,
    ALGORITHM_RELIABLE_LINK_PEEK_INVALID_ARGUMENT
} Algorithm_ReliableLink_PeekResult;

/* Caller-owned serialized state. No function is re-entrant. The encoded frame
 * remains in this object while a transport owns it, including after an ACK or
 * timeout, and is not reusable until TxComplete releases that ownership. */
typedef struct {
    Algorithm_ReliableLink_Config config;
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    uint32_t retry_due_offsets_ms[ALGORITHM_RELIABLE_LINK_MAX_RETRIES];
    size_t frame_length;
    uint32_t started_at_ms;
    uint32_t next_retry_due_offset_ms;
    uint16_t next_sequence;
    uint16_t outstanding_sequence;
    uint8_t retries_accepted;
    Algorithm_ReliableLink_State state;
    bool tx_in_flight;
    bool ack_latched;
    bool initialized;
} Algorithm_ReliableLink;

typedef struct {
    uint32_t last_session;
    uint16_t last_sequence;
    bool has_last;
} Algorithm_ReliableLink_Receiver;

typedef enum {
    ALGORITHM_RELIABLE_LINK_RECEIVE_DELIVER_AND_ACK = 0,
    ALGORITHM_RELIABLE_LINK_RECEIVE_DUPLICATE_REACK,
    ALGORITHM_RELIABLE_LINK_RECEIVE_INVALID_ARGUMENT
} Algorithm_ReliableLink_ReceiveDecision;

Algorithm_ReliableLink_Config Algorithm_ReliableLink_DefaultConfig(void);

/* total_timeout_ms must be nonzero and greater than max_retries. At most eight
 * retries are accepted so all retry offsets can be precomputed during Init.
 */
bool Algorithm_ReliableLink_Init(
    Algorithm_ReliableLink *link,
    const Algorithm_ReliableLink_Config *config,
    uint16_t initial_sequence);

/* Encodes one packet immediately. Begin is BUSY while a packet awaits ACK or
 * while the transport still owns the previous frame. A failed Begin consumes
 * no sequence number. */
Algorithm_ReliableLink_BeginResult Algorithm_ReliableLink_Begin(
    Algorithm_ReliableLink *link,
    const char *type,
    const uint8_t *payload,
    size_t payload_length);

/* Peek never consumes a send opportunity. The same due view is returned until
 * CommitTxAccepted succeeds. It may make a reached total deadline sticky by
 * moving the state to FAILED. The view points into link and must not be used
 * after a later successful Begin or Init. */
Algorithm_ReliableLink_PeekResult Algorithm_ReliableLink_PeekDueFrame(
    Algorithm_ReliableLink *link,
    uint32_t now_ms,
    Algorithm_ReliableLink_SendView *view);

/* Call only after a UART blocking write completed successfully or a DMA Start
 * returned OK. This is the operation that starts the deadline or consumes one
 * retry and marks the frame transport-owned. UART BUSY/errors must not call it.
 */
bool Algorithm_ReliableLink_CommitTxAccepted(
    Algorithm_ReliableLink *link, uint32_t now_ms);

/* Releases transport ownership after a blocking completion or the one DMA
 * callback. An ACK received while TX is in flight is latched; TxComplete then
 * finalizes it and makes the frame reusable. */
bool Algorithm_ReliableLink_TxComplete(Algorithm_ReliableLink *link);

/* ACK type/payload/session validation belongs to Device code. Matching ACKs at
 * or after the total deadline are rejected. A timely ACK may be latched while
 * TX owns the frame, but cannot release frame storage before TxComplete. */
bool Algorithm_ReliableLink_Acknowledge(
    Algorithm_ReliableLink *link,
    uint16_t sequence,
    uint32_t now_ms);

Algorithm_ReliableLink_State Algorithm_ReliableLink_GetState(
    const Algorithm_ReliableLink *link);
bool Algorithm_ReliableLink_HasOutstanding(
    const Algorithm_ReliableLink *link);
bool Algorithm_ReliableLink_HasTxInFlight(
    const Algorithm_ReliableLink *link);
bool Algorithm_ReliableLink_GetOutstandingSequence(
    const Algorithm_ReliableLink *link, uint16_t *sequence);

/* Receiver de-duplication key is (nonzero boot session, sequence). A peer must
 * choose a new session after every boot; sequence alone is not reboot-safe. */
void Algorithm_ReliableLink_Receiver_Init(
    Algorithm_ReliableLink_Receiver *receiver);
Algorithm_ReliableLink_ReceiveDecision
Algorithm_ReliableLink_Receiver_Observe(
    Algorithm_ReliableLink_Receiver *receiver,
    uint32_t session,
    uint16_t sequence);

#endif
