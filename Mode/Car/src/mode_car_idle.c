#include "mode_car_internal.h"

#include <stddef.h>

bool Mode_Car_Idle_Init(Mode_Car_IdleContext *context)
{
    if (context == NULL) {
        return false;
    }
    context->initialized = false;
    context->initialized = true;
    return true;
}

bool Mode_Car_Idle_Update(
    const Mode_Car_IdleContext *context,
    const Mode_Car_TimeInput *input,
    Mode_Car_Output *output)
{
    Mode_Car_Internal_StopOutput(output, MODE_CAR_COMPLETION_NONE);
    return (output != NULL) && (context != NULL) && context->initialized &&
        Mode_Car_Internal_ValidTime(input);
}
