#ifndef FREERTOS_SUPPORT_H
#define FREERTOS_SUPPORT_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_FreeRTOS_Fault;

bool FreeRTOS_Port_Validate(void);

#endif
