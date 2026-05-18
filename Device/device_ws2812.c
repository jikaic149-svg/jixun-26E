#include "device_ws2812.h"
#include "../BSP/bsp_ws2812.h"
#include "ti_msp_dl_config.h"

static uint8_t ws2812_buffer[WS2812_NUM_LEDS][3];

void Device_WS2812_Init(void) {
    BSP_WS2812_Init();
    Device_WS2812_Clear();
    Device_WS2812_Show();
}

void Device_WS2812_SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < WS2812_NUM_LEDS) {
        ws2812_buffer[index][0] = r;
        ws2812_buffer[index][1] = g;
        ws2812_buffer[index][2] = b;
    }
}

void Device_WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_buffer[i][0] = r;
        ws2812_buffer[i][1] = g;
        ws2812_buffer[i][2] = b;
    }
}

void Device_WS2812_Clear(void) {
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_buffer[i][0] = 0;
        ws2812_buffer[i][1] = 0;
        ws2812_buffer[i][2] = 0;
    }
}

void Device_WS2812_Show(void) {
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        BSP_WS2812_Send(ws2812_buffer[i][0], ws2812_buffer[i][1], ws2812_buffer[i][2]);
    }
    // 刷新后延迟产生 Reset 信号 (约 60us)
    delay_cycles(1920); // 32MHz 下: 32 * 60 = 1920 cycles
}
