#ifndef BSP_SOLENOID_H
#define BSP_SOLENOID_H

#include <stdbool.h>
#include <stddef.h>

/* Active-high GPIO controls for external low-side solenoid drivers. The BSP
 * only drives the transistor input; coil power never comes from an MCU pin. */
size_t BSP_Solenoid_Count(void);
bool BSP_Solenoid_Set(size_t index, bool on);
bool BSP_Solenoid_Get(size_t index, bool *on);

#endif
