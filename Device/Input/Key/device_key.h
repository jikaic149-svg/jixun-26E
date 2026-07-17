#ifndef DEVICE_KEY_DEVICE_KEY_H
#define DEVICE_KEY_DEVICE_KEY_H

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_KEY_EVENT_QUEUE_CAPACITY 4U

typedef enum {
    DEVICE_KEY_EVENT_SHORT_PRESS = 0,
    DEVICE_KEY_EVENT_LONG_PRESS,
    DEVICE_KEY_EVENT_DOUBLE_CLICK,
} Device_Key_Event;

typedef struct {
    uint32_t debounce_ms;
    uint32_t long_press_ms;
    uint32_t double_click_ms;
} Device_Key_Config;

/*
 * Instances are caller-owned so the driver needs neither allocation nor a
 * global singleton. Treat fields below as private; use the Device_Key_* API.
 */
typedef struct {
    Device_Key_Config config;
    uint32_t raw_changed_ms;
    uint32_t press_started_ms;
    uint32_t first_release_ms;
    Device_Key_Event event_queue[DEVICE_KEY_EVENT_QUEUE_CAPACITY];
    uint8_t event_head;
    uint8_t event_tail;
    uint8_t event_count;
    bool initialized;
    bool raw_pressed;
    bool stable_pressed;
    bool long_reported;
    bool second_click_candidate;
    bool pending_short;
    bool event_overflowed;
} Device_Key;

/* This state machine is intentionally single-context. Call Init, Update,
 * ReadEvent, and ClearEvents from one task/main-loop context, or serialize
 * every call externally. It is not safe to update it in an ISR while a task
 * reads its event queue. */

/*
 * Initialize one key instance. initial_pressed is taken as the initial stable
 * state; pass the current raw input level after converting it to pressed=true.
 * A key that is already held at initialization may therefore produce a long
 * press after config.long_press_ms. Timing thresholds must not exceed half of
 * the uint32_t timer range; long_press_ms and double_click_ms must be nonzero.
 */
bool Device_Key_Init(Device_Key *key,
                     const Device_Key_Config *config,
                     bool initial_pressed,
                     uint32_t now_ms);

/*
 * Feed the latest raw pressed state and a monotonic millisecond counter.
 * Call this often enough to observe every physical transition. uint32_t timer
 * rollover is supported. The return value is false only for an invalid key or
 * when this update could not enqueue every generated event.
 * A second debounced press beginning exactly double_click_ms after the first
 * release is part of the double-click window.
 */
bool Device_Key_Update(Device_Key *key, bool raw_pressed, uint32_t now_ms);

/* Return the debounced pressed state. An invalid instance reads as false. */
bool Device_Key_IsPressed(const Device_Key *key);

/* Read and remove one event in chronological order. */
bool Device_Key_ReadEvent(Device_Key *key, Device_Key_Event *event);

/* A sticky indication that at least one event was lost because the queue filled. */
bool Device_Key_EventOverflowed(const Device_Key *key);

/* Discard queued events and clear the sticky overflow indication. */
void Device_Key_ClearEvents(Device_Key *key);

#endif
