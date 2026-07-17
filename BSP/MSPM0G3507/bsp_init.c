#include "bsp_init.h"

#include "bsp_internal.h"
#include "bsp_led.h"
#include "bsp_rgb.h"

void BSP_Init(void)
{
    BSP_Encoder_Init();
    BSP_LED_Init();
    BSP_RGB_Init();
}
