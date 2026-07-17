#include "bsp_rgb.h"

#include "bsp_resource_map.h"

volatile uint8_t g_BSP_RGB_LastGRB[3] = {0U, 0U, 0U};
volatile uint32_t g_BSP_RGB_FrameCount = 0U;

void BSP_RGB_Init(void)
{
}

size_t BSP_RGB_Count(void)
{
    return BSP_RESOURCE_RGB_COUNT;
}

bool BSP_RGB_SetPixel(size_t index, BSP_RGB_Pixel color)
{
    (void) index;
    (void) color;
    return false;
}

bool BSP_RGB_Show(void)
{
    return false;
}
