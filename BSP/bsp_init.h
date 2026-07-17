#ifndef BSP_INIT_H
#define BSP_INIT_H

/* Initialize the resources explicitly enabled by the selected MCU BSP.
 * This is the only BSP-wide entry point; peripheral APIs remain in their
 * own bsp_*.h modules. */
void BSP_Init(void);

#endif
