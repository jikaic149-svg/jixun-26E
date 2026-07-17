#ifndef TASK_HEX25E_VISION_H
#define TASK_HEX25E_VISION_H

#include "device_vision.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    Device_Vision_Color color;
    Device_Vision_Shape shape;
    Device_Vision_Event last_event;
    uint32_t generation;
    uint32_t event_generation;
    uint32_t recovery_count;
    uint32_t rejected_frame_count;
    bool has_result;
    bool has_event;
    bool fault;
} Task_Hex25E_VisionStatus;

/* UART transport and protocol parsing only. Vision data never commands the
 * chassis until the Task explicitly adds a policy for it. */
bool Task_Hex25E_Vision_Init(void);
bool Task_Hex25E_Vision_Poll(uint32_t now_ms);
bool Task_Hex25E_Vision_GetStatus(Task_Hex25E_VisionStatus *status);
Device_Vision_BeginResult Task_Hex25E_Vision_BeginCommand(
    Device_Vision_Command command, Device_Vision_Request request);

#endif
