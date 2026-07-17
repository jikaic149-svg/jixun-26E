#include "bsp_led.h"

#include "bsp_resource_map.h"

void BSP_LED_Init(void)
{
}

size_t BSP_LED_Count(void)
{
    return BSP_RESOURCE_LED_COUNT;
}

bool BSP_LED_Set(size_t index, bool on)
{
    (void)index;
    (void)on;
    return false;
}
