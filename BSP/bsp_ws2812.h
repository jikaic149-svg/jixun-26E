#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdint.h>

void BSP_WS2812_Init(void);
void BSP_WS2812_Send(uint8_t r, uint8_t g, uint8_t b);

#endif // BSP_WS2812_H
