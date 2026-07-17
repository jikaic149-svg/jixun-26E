#include "bsp_encoder.h"

#include <stddef.h>

#include "bsp_resource_map.h"

#define BSP_ENCODER_COUNT 2U
#define BSP_ENCODER_HALF_RANGE UINT32_C(0x8000)
#define BSP_ENCODER_FULL_RANGE INT64_C(0x10000)

/* DL_TIMER_QEI_MODE_2_INPUT counts every valid A/B state transition:
 * A rising, A falling, B rising and B falling (x4 quadrature decoding). */

struct BSP_Encoder_Resource {
    GPTIMER_Regs *timer;
    int64_t cumulative;
    int64_t pending_delta;
    uint32_t previous_count;
};

static struct BSP_Encoder_Resource g_encoder[BSP_ENCODER_COUNT];

static struct BSP_Encoder_Resource *encoder_from_handle(
    BSP_Encoder_Handle encoder)
{
    for (size_t index = 0U; index < BSP_ENCODER_COUNT; ++index) {
        if (encoder == &g_encoder[index]) {
            return &g_encoder[index];
        }
    }
    return NULL;
}

static uint32_t encoder_read_count(
    const struct BSP_Encoder_Resource *encoder)
{
    return DL_Timer_getTimerCount(encoder->timer) &
        BSP_RESOURCE_ENCODER_COUNTER_MASK;
}

static void encoder_sample(struct BSP_Encoder_Resource *encoder)
{
    uint32_t current = encoder_read_count(encoder);
    uint32_t forward =
        (current - encoder->previous_count) &
        BSP_RESOURCE_ENCODER_COUNTER_MASK;
    int64_t delta = (forward < BSP_ENCODER_HALF_RANGE) ?
        (int64_t)forward :
        (int64_t)forward - BSP_ENCODER_FULL_RANGE;

    encoder->previous_count = current;
    encoder->cumulative += delta;
    encoder->pending_delta += delta;
}

void BSP_Encoder_Init(void)
{
    g_encoder[0] = (struct BSP_Encoder_Resource){
        .timer = BSP_RESOURCE_ENCODER_LEFT_TIMER,
    };
    g_encoder[1] = (struct BSP_Encoder_Resource){
        .timer = BSP_RESOURCE_ENCODER_RIGHT_TIMER,
    };
    g_encoder[0].previous_count = encoder_read_count(&g_encoder[0]);
    g_encoder[1].previous_count = encoder_read_count(&g_encoder[1]);
}

size_t BSP_Encoder_Count(void)
{
    return BSP_ENCODER_COUNT;
}

BSP_Encoder_Handle BSP_Encoder_GetHandle(size_t index)
{
    return (index < BSP_ENCODER_COUNT) ? &g_encoder[index] : NULL;
}

bool BSP_Encoder_GetSnapshot(
    BSP_Encoder_Handle encoder, int64_t *cumulative_count)
{
    struct BSP_Encoder_Resource *resource = encoder_from_handle(encoder);
    uint32_t primask;

    if ((resource == NULL) || (cumulative_count == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    encoder_sample(resource);
    *cumulative_count = resource->cumulative;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

bool BSP_Encoder_ReadDelta(BSP_Encoder_Handle encoder, int64_t *delta_count)
{
    struct BSP_Encoder_Resource *resource = encoder_from_handle(encoder);
    uint32_t primask;

    if ((resource == NULL) || (delta_count == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    encoder_sample(resource);
    *delta_count = resource->pending_delta;
    resource->pending_delta = 0;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}
