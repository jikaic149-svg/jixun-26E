#include "algorithm_spsc_ring.h"

#include <string.h>

static bool ring_is_valid(const Algorithm_SPSC_Ring *ring)
{
    return (ring != NULL) && (ring->storage != NULL) &&
        (ring->capacity > 0U) &&
        (ring->capacity <= ALGORITHM_SPSC_RING_MAX_CAPACITY);
}

static bool ring_producer_free(
    const Algorithm_SPSC_Ring *ring, size_t *producer_position, size_t *free)
{
    const size_t producer = atomic_load_explicit(
        &ring->producer_position, memory_order_relaxed);
    const size_t consumer = atomic_load_explicit(
        &ring->consumer_position, memory_order_acquire);
    const size_t used = producer - consumer;

    if (used > ring->capacity) {
        return false;
    }

    *producer_position = producer;
    *free = ring->capacity - used;
    return true;
}

static bool ring_consumer_used(
    const Algorithm_SPSC_Ring *ring, size_t *consumer_position, size_t *used)
{
    const size_t consumer = atomic_load_explicit(
        &ring->consumer_position, memory_order_relaxed);
    const size_t producer = atomic_load_explicit(
        &ring->producer_position, memory_order_acquire);
    const size_t occupied = producer - consumer;

    if (occupied > ring->capacity) {
        return false;
    }

    *consumer_position = consumer;
    *used = occupied;
    return true;
}

bool Algorithm_SPSC_Ring_Init(
    Algorithm_SPSC_Ring *ring, uint8_t *storage, size_t capacity)
{
    if (ring == NULL) {
        return false;
    }

    ring->storage = NULL;
    ring->capacity = 0U;
    atomic_init(&ring->producer_position, 0U);
    atomic_init(&ring->consumer_position, 0U);
    ring->producer_offset = 0U;
    ring->consumer_offset = 0U;

    if ((storage == NULL) || (capacity == 0U) ||
        (capacity > ALGORITHM_SPSC_RING_MAX_CAPACITY)) {
        return false;
    }

    ring->storage = storage;
    ring->capacity = capacity;
    return true;
}

size_t Algorithm_SPSC_Ring_Capacity(const Algorithm_SPSC_Ring *ring)
{
    return ring_is_valid(ring) ? ring->capacity : 0U;
}

size_t Algorithm_SPSC_Ring_Used(const Algorithm_SPSC_Ring *ring)
{
    size_t consumer_position;
    size_t used;

    if (!ring_is_valid(ring) ||
        !ring_consumer_used(ring, &consumer_position, &used)) {
        return 0U;
    }

    (void) consumer_position;
    return used;
}

size_t Algorithm_SPSC_Ring_Free(const Algorithm_SPSC_Ring *ring)
{
    size_t producer_position;
    size_t free;

    if (!ring_is_valid(ring) ||
        !ring_producer_free(ring, &producer_position, &free)) {
        return 0U;
    }

    (void) producer_position;
    return free;
}

bool Algorithm_SPSC_Ring_WriteByte(
    Algorithm_SPSC_Ring *ring, uint8_t byte)
{
    return Algorithm_SPSC_Ring_Write(ring, &byte, 1U);
}

bool Algorithm_SPSC_Ring_Write(
    Algorithm_SPSC_Ring *ring, const uint8_t *bytes, size_t length)
{
    size_t producer_position;
    size_t free;
    size_t offset;
    size_t first_length;

    if (!ring_is_valid(ring)) {
        return false;
    }
    if (length == 0U) {
        return true;
    }
    if ((bytes == NULL) ||
        !ring_producer_free(ring, &producer_position, &free) ||
        (length > free)) {
        return false;
    }

    offset = ring->producer_offset;
    first_length = ring->capacity - offset;
    if (first_length > length) {
        first_length = length;
    }

    memcpy(&ring->storage[offset], bytes, first_length);
    memcpy(ring->storage, &bytes[first_length], length - first_length);

    offset += length;
    if (offset >= ring->capacity) {
        offset -= ring->capacity;
    }
    ring->producer_offset = offset;
    atomic_store_explicit(
        &ring->producer_position, producer_position + length,
        memory_order_release);
    return true;
}

bool Algorithm_SPSC_Ring_ReadByte(
    Algorithm_SPSC_Ring *ring, uint8_t *byte)
{
    return (byte != NULL) && (Algorithm_SPSC_Ring_Read(ring, byte, 1U) == 1U);
}

size_t Algorithm_SPSC_Ring_Read(
    Algorithm_SPSC_Ring *ring, uint8_t *output, size_t output_capacity)
{
    size_t consumer_position;
    size_t used;
    size_t read_length;
    size_t offset;
    size_t first_length;

    if (!ring_is_valid(ring) || (output_capacity == 0U)) {
        return 0U;
    }
    if ((output == NULL) ||
        !ring_consumer_used(ring, &consumer_position, &used)) {
        return 0U;
    }

    read_length = output_capacity;
    if (read_length > used) {
        read_length = used;
    }
    if (read_length == 0U) {
        return 0U;
    }

    offset = ring->consumer_offset;
    first_length = ring->capacity - offset;
    if (first_length > read_length) {
        first_length = read_length;
    }

    memcpy(output, &ring->storage[offset], first_length);
    memcpy(&output[first_length], ring->storage, read_length - first_length);

    offset += read_length;
    if (offset >= ring->capacity) {
        offset -= ring->capacity;
    }
    ring->consumer_offset = offset;
    atomic_store_explicit(
        &ring->consumer_position, consumer_position + read_length,
        memory_order_release);
    return read_length;
}
