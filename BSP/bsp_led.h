#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>
#include <stdbool.h>

void BSP_LED_Toggle(uint8_t led_id);
void BSP_LED_Set(uint8_t led_id, bool state);

#endif // BSP_LED_H
