#include "mode_car_internal.h"

#include <stddef.h>

bool Mode_Car_Done_Init(Mode_Car_DoneContext *context)
{
    if (context == NULL) {
        return false;
    }
    context->initialized = false;
    context->initialized = true;
    return true;
}

bool Mode_Car_Done_Update(
    const Mode_Car_DoneContext *context,
    const Mode_Car_TimeInput *input,
    Mode_Car_Output *output)
{
    Mode_Car_Internal_StopOutput(
        output, MODE_CAR_COMPLETION_TERMINAL);
    return (output != NULL) && (context != NULL) && context->initialized &&
        Mode_Car_Internal_ValidTime(input);
}
