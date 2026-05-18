#include "bsp_key.h"
#include "ti_msp_dl_config.h"

bool BSP_Switch_Read(void) {
    // 按下为低电平，返回 true 表示按下
    return (DL_GPIO_readPins(GPIO_SWITCHES_PORT, GPIO_SWITCHES_USER_SWITCH_1_PIN) == 0);
}
