#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#ifndef TEMPLATE_FREERTOS_USE_HEAP_4
#define TEMPLATE_FREERTOS_USE_HEAP_4             0
#endif
#ifndef TEMPLATE_FREERTOS_USE_TIMERS
#define TEMPLATE_FREERTOS_USE_TIMERS             0
#endif
#ifndef TEMPLATE_FREERTOS_USE_EVENT_GROUPS
#define TEMPLATE_FREERTOS_USE_EVENT_GROUPS       0
#endif
#ifndef TEMPLATE_FREERTOS_USE_STREAM_BUFFERS
#define TEMPLATE_FREERTOS_USE_STREAM_BUFFERS     0
#endif
#ifndef TEMPLATE_FREERTOS_USE_TRACE
#define TEMPLATE_FREERTOS_USE_TRACE              0
#endif

#define configCPU_CLOCK_HZ                      ((uint32_t) CPUCLK_FREQ)
#define configTICK_RATE_HZ                       1000U
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                  0
#define configMAX_PRIORITIES                     5U
#define configMINIMAL_STACK_SIZE                 128U
#define configIDLE_TASK_STACK_DEPTH              configMINIMAL_STACK_SIZE
#define configMAX_TASK_NAME_LEN                  16U
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                  1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1U
#define configQUEUE_REGISTRY_SIZE                4U
#define configENABLE_BACKWARD_COMPATIBILITY      1
#define configUSE_MINI_LIST_ITEM                 1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
#define configUSE_NEWLIB_REENTRANT               0

#define configUSE_TIMERS                         TEMPLATE_FREERTOS_USE_TIMERS
#define configTIMER_TASK_PRIORITY                3U
#define configTIMER_TASK_STACK_DEPTH             192U
#define configTIMER_QUEUE_LENGTH                 8U
#define configUSE_EVENT_GROUPS                   TEMPLATE_FREERTOS_USE_EVENT_GROUPS
#define configUSE_STREAM_BUFFERS                 TEMPLATE_FREERTOS_USE_STREAM_BUFFERS

#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         TEMPLATE_FREERTOS_USE_HEAP_4
#define configTOTAL_HEAP_SIZE                    (8U * 1024U)
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configHEAP_CLEAR_MEMORY_ON_FREE          1

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             TEMPLATE_FREERTOS_USE_HEAP_4
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_SB_COMPLETED_CALLBACK          0

#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 TEMPLATE_FREERTOS_USE_TRACE
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          1

#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_QUEUE_SETS                     0
#define configUSE_APPLICATION_TASK_TAG           0

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xResumeFromISR                   1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           0
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           TEMPLATE_FREERTOS_USE_TIMERS
#define INCLUDE_xTaskAbortDelay                  1
#define INCLUDE_xTaskGetHandle                   0
#define INCLUDE_xTaskResumeFromISR               1
#define INCLUDE_xSemaphoreGetMutexHolder         1

#define configENABLE_MPU                         0
#define configUSE_MPU_WRAPPERS_V1                0
#define configPRIO_BITS                          2U
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  3U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1U
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))

#define configASSERT_DEFINED 1
#define configASSERT(x)                  \
    do {                                 \
        if ((x) == 0) {                  \
            taskDISABLE_INTERRUPTS();    \
            for (;;) {                   \
                __asm volatile("nop");  \
            }                            \
        }                                \
    } while (0)

#endif
