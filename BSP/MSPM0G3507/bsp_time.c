#include "bsp_time.h"

#include "bsp_resource_map.h"

void BSP_Time_DelayMs(uint32_t milliseconds)
{
    while (milliseconds > 0U) {
        DL_Common_delayCycles(BSP_RESOURCE_CPU_CLOCK_HZ / 1000U);
        --milliseconds;
    }
}
