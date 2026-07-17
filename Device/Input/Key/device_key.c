#include "device_key.h"

#include <stddef.h>

static uint32_t Device_Key_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static bool Device_Key_QueueEvent(Device_Key *key, Device_Key_Event event)
{
    if (key->event_count >= DEVICE_KEY_EVENT_QUEUE_CAPACITY) {
        key->event_overflowed = true;
        return false;
    }

    key->event_queue[key->event_tail] = event;
    key->event_tail =
        (uint8_t)((key->event_tail + 1U) % DEVICE_KEY_EVENT_QUEUE_CAPACITY);
    ++key->event_count;
    return true;
}

static bool Device_Key_HandlePressedEdge(Device_Key *key,
                                         uint32_t edge_time_ms)
{
    bool complete = true;

    key->press_started_ms = edge_time_ms;
    key->long_reported = false;
    key->second_click_candidate = false;

    if (key->pending_short) {
        const uint32_t gap_ms =
            Device_Key_ElapsedMs(edge_time_ms, key->first_release_ms);

        if (gap_ms <= key->config.double_click_ms) {
            key->pending_short = false;
            key->second_click_candidate = true;
        } else {
            complete = Device_Key_QueueEvent(
                key, DEVICE_KEY_EVENT_SHORT_PRESS);
            key->pending_short = false;
        }
    }

    return complete;
}

static bool Device_Key_ReportLongPress(Device_Key *key)
{
    bool complete = true;

    if (key->second_click_candidate) {
        complete = Device_Key_QueueEvent(
            key, DEVICE_KEY_EVENT_SHORT_PRESS);
        key->second_click_candidate = false;
    }

    complete = Device_Key_QueueEvent(key, DEVICE_KEY_EVENT_LONG_PRESS) &&
               complete;
    key->long_reported = true;
    return complete;
}

static bool Device_Key_HandleReleasedEdge(Device_Key *key,
                                          uint32_t edge_time_ms)
{
    bool complete = true;

    /* Classify from debounced edge times. A release exactly on the threshold
     * is a long press, independent of whether Update() happened immediately
     * before that edge. */
    if (!key->long_reported &&
        (Device_Key_ElapsedMs(edge_time_ms, key->press_started_ms) >=
         key->config.long_press_ms)) {
        complete = Device_Key_ReportLongPress(key);
    }

    if (key->long_reported) {
        key->long_reported = false;
        return complete;
    }

    if (key->second_click_candidate) {
        key->second_click_candidate = false;
        return Device_Key_QueueEvent(key, DEVICE_KEY_EVENT_DOUBLE_CLICK);
    }

    key->pending_short = true;
    key->first_release_ms = edge_time_ms;
    return true;
}

bool Device_Key_Init(Device_Key *key,
                     const Device_Key_Config *config,
                     bool initial_pressed,
                     uint32_t now_ms)
{
    if ((key == NULL) || (config == NULL) ||
        (config->long_press_ms == 0U) ||
        (config->double_click_ms == 0U) ||
        (config->debounce_ms > (UINT32_MAX / 2U)) ||
        (config->long_press_ms > (UINT32_MAX / 2U)) ||
        (config->double_click_ms > (UINT32_MAX / 2U))) {
        return false;
    }

    *key = (Device_Key){0};
    key->config = *config;
    key->raw_changed_ms = now_ms;
    key->press_started_ms = now_ms;
    key->initialized = true;
    key->raw_pressed = initial_pressed;
    key->stable_pressed = initial_pressed;
    return true;
}

bool Device_Key_Update(Device_Key *key, bool raw_pressed, uint32_t now_ms)
{
    bool complete = true;

    if ((key == NULL) || !key->initialized) {
        return false;
    }

    if (raw_pressed != key->raw_pressed) {
        key->raw_pressed = raw_pressed;
        key->raw_changed_ms = now_ms;
    }

    if ((key->raw_pressed != key->stable_pressed) &&
        (Device_Key_ElapsedMs(now_ms, key->raw_changed_ms) >=
         key->config.debounce_ms)) {
        const uint32_t edge_time_ms =
            key->raw_changed_ms + key->config.debounce_ms;

        key->stable_pressed = key->raw_pressed;
        if (key->stable_pressed) {
            complete = Device_Key_HandlePressedEdge(key, edge_time_ms);
        } else {
            complete = Device_Key_HandleReleasedEdge(key, edge_time_ms);
        }
    }

    if (key->stable_pressed && !key->long_reported &&
        (Device_Key_ElapsedMs(now_ms, key->press_started_ms) >=
         key->config.long_press_ms)) {
        complete = Device_Key_ReportLongPress(key) && complete;
    }

    if (key->pending_short &&
        (Device_Key_ElapsedMs(now_ms, key->first_release_ms) >=
         key->config.double_click_ms)) {
        complete = Device_Key_QueueEvent(key, DEVICE_KEY_EVENT_SHORT_PRESS) &&
                   complete;
        key->pending_short = false;
    }

    return complete;
}

bool Device_Key_IsPressed(const Device_Key *key)
{
    return (key != NULL) && key->initialized && key->stable_pressed;
}

bool Device_Key_ReadEvent(Device_Key *key, Device_Key_Event *event)
{
    if ((key == NULL) || (event == NULL) || !key->initialized ||
        (key->event_count == 0U)) {
        return false;
    }

    *event = key->event_queue[key->event_head];
    key->event_head =
        (uint8_t)((key->event_head + 1U) % DEVICE_KEY_EVENT_QUEUE_CAPACITY);
    --key->event_count;
    return true;
}

bool Device_Key_EventOverflowed(const Device_Key *key)
{
    return (key != NULL) && key->initialized && key->event_overflowed;
}

void Device_Key_ClearEvents(Device_Key *key)
{
    if ((key == NULL) || !key->initialized) {
        return;
    }

    key->event_head = 0U;
    key->event_tail = 0U;
    key->event_count = 0U;
    key->event_overflowed = false;
}
