#ifndef BSP_INIT_H
#define BSP_INIT_H

#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_ws2812.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "bsp_qei.h"

/* 
 * BSP 层 (底层板级支持) 
 */

void BSP_Init(void);

#endif // BSP_INIT_H
