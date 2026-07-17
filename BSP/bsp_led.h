#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stddef.h>

/* Board-local discrete LEDs. Electrical polarity stays inside the BSP. */
void BSP_LED_Init(void);
size_t BSP_LED_Count(void);
bool BSP_LED_Set(size_t index, bool on);

#endif
