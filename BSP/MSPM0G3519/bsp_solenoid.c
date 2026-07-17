#include "bsp_solenoid.h"

#include "bsp_resource_map.h"

#include <stdint.h>

static bool g_solenoid_on[BSP_RESOURCE_SOLENOID_COUNT];

void BSP_Solenoid_Init(void)
{
    DL_GPIO_clearPins(
        BSP_RESOURCE_SOLENOID_PORT, BSP_RESOURCE_SOLENOID_PIN);
    g_solenoid_on[0] = false;
}

size_t BSP_Solenoid_Count(void)
{
    return BSP_RESOURCE_SOLENOID_COUNT;
}

bool BSP_Solenoid_Set(size_t index, bool on)
{
    if (index >= BSP_RESOURCE_SOLENOID_COUNT) {
        return false;
    }
    if (on) {
        DL_GPIO_setPins(
            BSP_RESOURCE_SOLENOID_PORT, BSP_RESOURCE_SOLENOID_PIN);
    }
    else {
        DL_GPIO_clearPins(
            BSP_RESOURCE_SOLENOID_PORT, BSP_RESOURCE_SOLENOID_PIN);
    }
    g_solenoid_on[index] = on;
    return true;
}

bool BSP_Solenoid_Get(size_t index, bool *on)
{
    if ((index >= BSP_RESOURCE_SOLENOID_COUNT) || (on == NULL)) {
        return false;
    }
    *on = g_solenoid_on[index];
    return true;
}
