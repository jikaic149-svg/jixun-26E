#include "bsp_init.h"
#include "bsp_motor.h"

void BSP_Init(void) {
    // 触发各子模块的初始化（部分引脚在 SYSCFG 中已自动初始化，此处只启动外设或中断）
    BSP_UART1_Init();
    BSP_UART2_Init();
    BSP_Motor_Init();  // 该函数内部会自动调用 BSP_PWM_Init()
    BSP_QEI_Init();
    BSP_WS2812_Init();
}
