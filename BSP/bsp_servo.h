#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct BSP_Servo_Resource *BSP_Servo_Handle;

/* A servo backend owns one fixed-frequency hardware PWM output. Disabling an
 * output stops its timer and actively holds the signal pin low. */
size_t BSP_Servo_Count(void);
BSP_Servo_Handle BSP_Servo_GetHandle(size_t index);
bool BSP_Servo_SetEnabled(BSP_Servo_Handle servo, bool enabled);
bool BSP_Servo_IsEnabled(BSP_Servo_Handle servo, bool *enabled);

/* Pulse width and frame period are expressed in microseconds. SetPulseUs may
 * be called while disabled; the value takes effect on the next enable. */
bool BSP_Servo_SetPulseUs(BSP_Servo_Handle servo, uint32_t pulse_us);
bool BSP_Servo_GetPulseUs(BSP_Servo_Handle servo, uint32_t *pulse_us);
bool BSP_Servo_GetFramePeriodUs(
    BSP_Servo_Handle servo, uint32_t *period_us);

#endif
