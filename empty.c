#include "ti_msp_dl_config.h"
#include "BSP/bsp_init.h"
#include "Device/device_init.h"
#include "APP/app_main.h"

int main(void)
{
    SYSCFG_DL_init();
    BSP_Init();
    Device_Init();
    App_Init();   /* 启动 FreeRTOS 调度，不再返回 */
    return 0;
}
