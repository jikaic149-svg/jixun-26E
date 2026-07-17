#include "FreeRTOS.h"
#include "task.h"

#include "freertos_support.h"

volatile uint32_t g_FreeRTOS_Fault = 0U;

bool FreeRTOS_Port_Validate(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED;
}

void vApplicationGetIdleTaskMemory(
    StaticTask_t **task_tcb,
    StackType_t **task_stack,
    configSTACK_DEPTH_TYPE *task_stack_size)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

    *task_tcb = &idle_tcb;
    *task_stack = idle_stack;
    *task_stack_size = configMINIMAL_STACK_SIZE;
}

#if configUSE_TIMERS == 1
void vApplicationGetTimerTaskMemory(
    StaticTask_t **task_tcb,
    StackType_t **task_stack,
    configSTACK_DEPTH_TYPE *task_stack_size)
{
    static StaticTask_t timer_tcb;
    static StackType_t timer_stack[configTIMER_TASK_STACK_DEPTH];

    *task_tcb = &timer_tcb;
    *task_stack = timer_stack;
    *task_stack_size = configTIMER_TASK_STACK_DEPTH;
}
#endif

#if configSUPPORT_DYNAMIC_ALLOCATION == 1
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    g_FreeRTOS_Fault = 1U;
    for (;;) {
        __asm volatile("nop");
    }
}
#endif

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void) task;
    (void) task_name;

    taskDISABLE_INTERRUPTS();
    g_FreeRTOS_Fault = 2U;
    for (;;) {
        __asm volatile("nop");
    }
}
