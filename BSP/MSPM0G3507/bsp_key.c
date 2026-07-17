#include "bsp_key.h"

#include <stddef.h>
#include <stdint.h>

#include "bsp_resource_map.h"

struct BSP_Key_Resource {
    uint8_t unused;
};

static const struct BSP_Key_Resource g_user_key = {0U};

size_t BSP_Key_Count(void)
{
    return 1U;
}

BSP_Key_Handle BSP_Key_GetHandle(size_t index)
{
    return (index == 0U) ? &g_user_key : NULL;
}

bool BSP_Key_Read(BSP_Key_Handle key, bool *pressed)
{
    if ((key != &g_user_key) || (pressed == NULL)) {
        return false;
    }
    *pressed = (DL_GPIO_readPins(
        BSP_RESOURCE_USER_KEY_PORT, BSP_RESOURCE_USER_KEY_PIN) == 0U);
    return true;
}
