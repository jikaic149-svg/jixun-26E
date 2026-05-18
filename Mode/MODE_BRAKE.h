#ifndef __MODE_BRAKE_H
#define __MODE_BRAKE_H

#include <stdint.h>

void    MODE_Brake_Init(void);
void    MODE_Brake_Exec(void);
uint8_t MODE_Brake_IsDone(void);

#endif /* __MODE_BRAKE_H */
