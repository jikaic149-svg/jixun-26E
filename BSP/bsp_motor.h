#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stddef.h>

#include "bsp_pwm.h"

typedef const struct BSP_Motor_Resource *BSP_Motor_Handle;

typedef enum {
    BSP_MOTOR_MODE_COAST = 0,
    BSP_MOTOR_MODE_BRAKE,
    BSP_MOTOR_MODE_FORWARD,
    BSP_MOTOR_MODE_REVERSE,
} BSP_Motor_Mode;

typedef struct {
    BSP_Motor_Mode mode;
    BSP_PWM_Duty duty;
} BSP_Motor_State;

/* PWM channels, direction pins, polarity, coast and brake behavior all belong
 * to the selected MCU BSP. No physical pin or timer appears in this API. */
size_t BSP_Motor_Count(void);
BSP_Motor_Handle BSP_Motor_GetHandle(size_t index);

/* Apply mode and duty as one backend operation so direction cannot change
 * through an unsafe energized intermediate state. */
bool BSP_Motor_Apply(BSP_Motor_Handle motor, BSP_Motor_State state);
bool BSP_Motor_GetState(BSP_Motor_Handle motor, BSP_Motor_State *state);

#endif
