#include "bsp_ws2812.h"
#include "ti_msp_dl_config.h"

// MSPM0G3519 @ 32MHz
// 1 cycle = 31.25 ns
void BSP_WS2812_Init(void)
{
    // PA29 - 假设对应 IOMUX_PINCM4
    DL_GPIO_initDigitalOutput(IOMUX_PINCM4);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_29);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_29);
}

#define WS_HIGH()  (GPIOA->DOUTSET31_0 = DL_GPIO_PIN_29)
#define WS_LOW()   (GPIOA->DOUTCLR31_0 = DL_GPIO_PIN_29)

// 32MHz 下：1 cycle = 31.25 ns
// 发送 0: T0H=400ns(~13 ticks), T0L=850ns(~27 ticks)
// 发送 1: T1H=800ns(~26 ticks), T1L=450ns(~14 ticks)
// 极端缩短 T0H，防止因为编译器未开启优化（-O0）导致的指令插空和地址重载开销。
// 即使只有一个 NOP，在 -O0 级别的取指、译码开销下，也足够产生 300ns 的高电平了。
static inline void WS2812_SendZero(void) {
    GPIOA->DOUTSET31_0 = DL_GPIO_PIN_29;
    __NOP(); __NOP(); // 仅保留 2 个 NOP，极度压榨高电平时间
    GPIOA->DOUTCLR31_0 = DL_GPIO_PIN_29;
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
}

static inline void WS2812_SendOne(void) {
    GPIOA->DOUTSET31_0 = DL_GPIO_PIN_29;
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
    GPIOA->DOUTCLR31_0 = DL_GPIO_PIN_29;
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
}

void BSP_WS2812_Send(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812 数据格式为 GRB 顺序，高位在前
    uint32_t color = (g << 16) | (r << 8) | b;
    
    __disable_irq(); // 关闭中断，保证严格时序
    for (int i = 0; i < 24; i++) {
        if (color & 0x800000) {
            WS2812_SendOne();
        } else {
            WS2812_SendZero();
        }
        color <<= 1;
    }
    __enable_irq();
}
