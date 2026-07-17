#include "bsp_led.h"

#include "bsp_resource_map.h"

#include <stdint.h>

static const uint32_t g_led_pin[BSP_RESOURCE_LED_COUNT] = {
    BSP_RESOURCE_LED_0_PIN,
    BSP_RESOURCE_LED_1_PIN,
};

void BSP_LED_Init(void)
{
    /* LED1/LED2 are wired from 3V3 through a resistor into the MCU pin, so
     * driving high is off and driving low sinks current to turn them on. */
    DL_GPIO_setPins(BSP_RESOURCE_LED_PORT,
        BSP_RESOURCE_LED_0_PIN | BSP_RESOURCE_LED_1_PIN);
}

size_t BSP_LED_Count(void)
{
    return BSP_RESOURCE_LED_COUNT;
}

bool BSP_LED_Set(size_t index, bool on)
{
    if (index >= BSP_RESOURCE_LED_COUNT) {
        return false;
    }
    if (on) {
        DL_GPIO_clearPins(BSP_RESOURCE_LED_PORT, g_led_pin[index]);
    }
    else {
        DL_GPIO_setPins(BSP_RESOURCE_LED_PORT, g_led_pin[index]);
    }
    return true;
}
