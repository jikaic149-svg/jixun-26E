#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_PWM_DUTY_MIN   0U
#define BSP_PWM_DUTY_MAX   10000U

/* Logical duty in ten-thousandths: 0 is 0%, 10000 is 100%. */
typedef uint32_t BSP_PWM_Duty;

/*
 * A PWM handle identifies one logical output. Its pin, polarity, timer,
 * channel, and update backend are private to the selected MCU BSP.
 */
typedef const struct BSP_PWM_Resource *BSP_PWM_Handle;

/* Return the number of logical PWM outputs provided by this BSP; zero is valid. */
size_t BSP_PWM_Count(void);

/*
 * Return a program-lifetime handle for index, or NULL when index is outside
 * BSP_PWM_Count(). Indices describe BSP resources, not pins or product roles.
 */
BSP_PWM_Handle BSP_PWM_GetHandle(size_t index);

/*
 * Set the logical active-time duty after BSP_Init(). The BSP applies any
 * required electrical inversion. Values above BSP_PWM_DUTY_MAX are rejected,
 * never clamped. Return false for an invalid handle, an out-of-range duty, or
 * a backend update failure; a failed call leaves the prior duty in effect.
 */
bool BSP_PWM_SetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty duty);

/*
 * Read the last successfully applied logical duty. Return false for a
 * NULL/foreign handle or NULL output pointer. On failure, duty is not modified.
 */
bool BSP_PWM_GetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty *duty);

/*
 * Read the configured output frequency in hertz. This contract intentionally
 * has no runtime frequency setter. Return false for a NULL/foreign handle or
 * NULL output pointer. On failure, frequency_hz is not modified.
 */
bool BSP_PWM_GetFrequencyHz(BSP_PWM_Handle pwm, uint32_t *frequency_hz);

#endif
