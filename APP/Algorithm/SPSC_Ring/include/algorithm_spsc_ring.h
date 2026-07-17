#ifndef ALGORITHM_SPSC_RING_H
#define ALGORITHM_SPSC_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

/* A byte ring with exactly one producer and exactly one consumer.
 *
 * The caller owns the storage and must keep it alive until the ring is no
 * longer used. The producer is the only context allowed to call WriteByte()
 * and Write(). The consumer is the only context allowed to call ReadByte()
 * and Read(). Initialization must happen while both contexts are stopped.
 *
 * C11 acquire/release atomic loads/stores publish bytes from producer to
 * consumer without an RTOS or MCU-specific critical-section API. The locked
 * ARMv6-M toolchain emits aligned LDR/STR plus DMB and no libatomic helper.
 * Do not replace the owned load/store update with atomic read-modify-write
 * operations unless the target toolchain is re-audited.
 */
typedef struct {
    uint8_t *storage;
    size_t capacity;
    atomic_size_t producer_position;
    atomic_size_t consumer_position;
    size_t producer_offset;
    size_t consumer_offset;
} Algorithm_SPSC_Ring;

/* The bound keeps modular producer-consumer subtraction unambiguous. */
#define ALGORITHM_SPSC_RING_MAX_CAPACITY (SIZE_MAX / 2U)

/* Initialize an empty ring over caller-provided storage.
 * Return false for a null pointer, zero capacity, or an excessive capacity.
 * A non-null ring is left in a safe, invalid state when initialization fails.
 */
bool Algorithm_SPSC_Ring_Init(
    Algorithm_SPSC_Ring *ring, uint8_t *storage, size_t capacity);

/* Return the immutable storage capacity, or zero for an invalid ring. */
size_t Algorithm_SPSC_Ring_Capacity(const Algorithm_SPSC_Ring *ring);

/* Return a bounded occupancy snapshot. Used() is naturally a consumer-side
 * query; Free() is naturally a producer-side query. Their values may become
 * stale immediately when the other context makes progress.
 */
size_t Algorithm_SPSC_Ring_Used(const Algorithm_SPSC_Ring *ring);
size_t Algorithm_SPSC_Ring_Free(const Algorithm_SPSC_Ring *ring);

/* Append one byte. Return false and leave the ring unchanged when full. */
bool Algorithm_SPSC_Ring_WriteByte(
    Algorithm_SPSC_Ring *ring, uint8_t byte);

/* Append the complete byte sequence as one commit. If free space is smaller
 * than length, return false without writing any byte. A zero-length write is
 * valid and does not require bytes to be non-null. The input must not overlap
 * the ring's storage.
 */
bool Algorithm_SPSC_Ring_Write(
    Algorithm_SPSC_Ring *ring, const uint8_t *bytes, size_t length);

/* Remove one byte. Return false and leave the ring unchanged when empty. */
bool Algorithm_SPSC_Ring_ReadByte(
    Algorithm_SPSC_Ring *ring, uint8_t *byte);

/* Remove up to output_capacity bytes and return the count removed. A
 * zero-length read is valid and does not require output to be non-null. The
 * output must not overlap the ring's storage.
 */
size_t Algorithm_SPSC_Ring_Read(
    Algorithm_SPSC_Ring *ring, uint8_t *output, size_t output_capacity);

#endif
