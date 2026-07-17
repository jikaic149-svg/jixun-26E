#include "bsp_servo.h"

#include "bsp_resource_map.h"

#include <stddef.h>

#define BSP_SERVO_OUTPUT_COUNT 1U
#define BSP_SERVO_INITIAL_PULSE_US UINT32_C(1500)

struct BSP_Servo_Resource {
    uint32_t pulse_us;
    bool enabled;
};

static struct BSP_Servo_Resource g_servo[BSP_SERVO_OUTPUT_COUNT] = {
    {BSP_SERVO_INITIAL_PULSE_US, false},
};

static struct BSP_Servo_Resource *servo_from_handle(
    BSP_Servo_Handle servo)
{
    for (size_t index = 0U; index < BSP_SERVO_OUTPUT_COUNT; ++index) {
        if (servo == &g_servo[index]) {
            return &g_servo[index];
        }
    }
    return NULL;
}

static void servo_pin_low(void)
{
    DL_GPIO_initDigitalOutput(BSP_RESOURCE_SERVO_IOMUX);
    DL_GPIO_clearPins(BSP_RESOURCE_SERVO_PORT, BSP_RESOURCE_SERVO_PIN);
    DL_GPIO_enableOutput(BSP_RESOURCE_SERVO_PORT, BSP_RESOURCE_SERVO_PIN);
}

static void servo_pin_pwm(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        BSP_RESOURCE_SERVO_IOMUX, BSP_RESOURCE_SERVO_IOMUX_FUNC);
    DL_GPIO_enableOutput(BSP_RESOURCE_SERVO_PORT, BSP_RESOURCE_SERVO_PIN);
}

static void servo_apply_pulse(uint32_t pulse_us)
{
    const uint32_t period_ticks = BSP_RESOURCE_SERVO_PERIOD_US *
        BSP_RESOURCE_SERVO_TICKS_PER_US;
    const uint32_t pulse_ticks = pulse_us *
        BSP_RESOURCE_SERVO_TICKS_PER_US;

    DL_TimerG_setCaptureCompareValue(BSP_RESOURCE_SERVO_TIMER,
        period_ticks - pulse_ticks, BSP_RESOURCE_SERVO_CC);
}

void BSP_Servo_Init(void)
{
    DL_TimerG_stopCounter(BSP_RESOURCE_SERVO_TIMER);
    DL_TimerG_setCaptCompUpdateMethod(BSP_RESOURCE_SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, BSP_RESOURCE_SERVO_CC);
    servo_apply_pulse(g_servo[0].pulse_us);
    /* Apply pulse changes only at a PWM frame boundary. Immediate compare
     * updates can truncate or extend the active 20 ms frame and make a hobby
     * servo twitch while its target is changing. */
    DL_TimerG_setCaptCompUpdateMethod(BSP_RESOURCE_SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, BSP_RESOURCE_SERVO_CC);
    servo_pin_low();
}

size_t BSP_Servo_Count(void)
{
    return BSP_SERVO_OUTPUT_COUNT;
}

BSP_Servo_Handle BSP_Servo_GetHandle(size_t index)
{
    return (index < BSP_SERVO_OUTPUT_COUNT) ? &g_servo[index] : NULL;
}

bool BSP_Servo_SetEnabled(BSP_Servo_Handle servo, bool enabled)
{
    struct BSP_Servo_Resource *resource = servo_from_handle(servo);

    if (resource == NULL) {
        return false;
    }
    if (resource->enabled == enabled) {
        return true;
    }
    if (!enabled) {
        resource->enabled = false;
        DL_TimerG_stopCounter(BSP_RESOURCE_SERVO_TIMER);
        servo_pin_low();
        return true;
    }

    /* The counter is stopped, so seed the active compare register directly;
     * subsequent running updates return to zero-event shadow transfers. */
    DL_TimerG_setCaptCompUpdateMethod(BSP_RESOURCE_SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, BSP_RESOURCE_SERVO_CC);
    servo_apply_pulse(resource->pulse_us);
    DL_TimerG_setCaptCompUpdateMethod(BSP_RESOURCE_SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, BSP_RESOURCE_SERVO_CC);
    DL_TimerG_setTimerCount(BSP_RESOURCE_SERVO_TIMER,
        BSP_RESOURCE_SERVO_PERIOD_US * BSP_RESOURCE_SERVO_TICKS_PER_US);
    servo_pin_pwm();
    DL_TimerG_startCounter(BSP_RESOURCE_SERVO_TIMER);
    resource->enabled = true;
    return true;
}

bool BSP_Servo_IsEnabled(BSP_Servo_Handle servo, bool *enabled)
{
    struct BSP_Servo_Resource *resource = servo_from_handle(servo);

    if ((resource == NULL) || (enabled == NULL)) {
        return false;
    }
    *enabled = resource->enabled;
    return true;
}

bool BSP_Servo_SetPulseUs(BSP_Servo_Handle servo, uint32_t pulse_us)
{
    struct BSP_Servo_Resource *resource = servo_from_handle(servo);

    if ((resource == NULL) || (pulse_us == 0U) ||
        (pulse_us >= BSP_RESOURCE_SERVO_PERIOD_US)) {
        return false;
    }
    servo_apply_pulse(pulse_us);
    resource->pulse_us = pulse_us;
    return true;
}

bool BSP_Servo_GetPulseUs(BSP_Servo_Handle servo, uint32_t *pulse_us)
{
    struct BSP_Servo_Resource *resource = servo_from_handle(servo);

    if ((resource == NULL) || (pulse_us == NULL)) {
        return false;
    }
    *pulse_us = resource->pulse_us;
    return true;
}

bool BSP_Servo_GetFramePeriodUs(
    BSP_Servo_Handle servo, uint32_t *period_us)
{
    if ((servo_from_handle(servo) == NULL) || (period_us == NULL)) {
        return false;
    }
    *period_us = BSP_RESOURCE_SERVO_PERIOD_US;
    return true;
}
