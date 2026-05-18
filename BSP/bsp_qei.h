#ifndef BSP_QEI_H
#define BSP_QEI_H

#include <stdint.h>

void BSP_QEI_Init(void);
int16_t BSP_QEI_GetDeltaL(void);
int16_t BSP_QEI_GetDeltaR(void);

#endif // BSP_QEI_H
