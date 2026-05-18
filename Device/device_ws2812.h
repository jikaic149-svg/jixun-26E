#ifndef DEVICE_WS2812_H
#define DEVICE_WS2812_H

#include <stdint.h>

#define WS2812_NUM_LEDS 1 // 根据实际串联灯珠数量修改

void Device_WS2812_Init(void);
void Device_WS2812_SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void Device_WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void Device_WS2812_Clear(void);
void Device_WS2812_Show(void);

#endif
