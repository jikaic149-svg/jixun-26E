#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/* Current bare-metal backend delays without owning or reconfiguring SysTick.
 * Behavior after a future scheduler starts is intentionally not fixed yet. */
void BSP_Time_DelayMs(uint32_t milliseconds);

#endif
