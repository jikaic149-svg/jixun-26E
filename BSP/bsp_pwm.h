#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

void BSP_PWM_Init(void);
void BSP_PWM_SetDutyL(uint32_t duty);
void BSP_PWM_SetDutyR(uint32_t duty);

#endif // BSP_PWM_H
