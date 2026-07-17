#include "bsp_pwm.h"

#include <stddef.h>

#include "bsp_resource_map.h"

#define BSP_PWM_OUTPUT_COUNT 2U
#define BSP_PWM_FREQUENCY_HZ 20000U

struct BSP_PWM_Resource {
    DL_TIMER_CC_INDEX cc_index;
    BSP_PWM_Duty duty;
};

static struct BSP_PWM_Resource g_pwm[BSP_PWM_OUTPUT_COUNT] = {
    {BSP_RESOURCE_MOTOR_LEFT_CC, 0U},
    {BSP_RESOURCE_MOTOR_RIGHT_CC, 0U},
};

static struct BSP_PWM_Resource *pwm_from_handle(BSP_PWM_Handle pwm)
{
    for (size_t index = 0U; index < BSP_PWM_OUTPUT_COUNT; ++index) {
        if (pwm == &g_pwm[index]) {
            return &g_pwm[index];
        }
    }
    return NULL;
}

void BSP_PWM_Init(void)
{
    for (size_t index = 0U; index < BSP_PWM_OUTPUT_COUNT; ++index) {
        DL_TimerA_setCaptureCompareValue(BSP_RESOURCE_MOTOR_PWM_TIMER,
            BSP_RESOURCE_MOTOR_PWM_PERIOD, g_pwm[index].cc_index);
        g_pwm[index].duty = 0U;
    }
    DL_TimerA_startCounter(BSP_RESOURCE_MOTOR_PWM_TIMER);
}

size_t BSP_PWM_Count(void)
{
    return BSP_PWM_OUTPUT_COUNT;
}

BSP_PWM_Handle BSP_PWM_GetHandle(size_t index)
{
    return (index < BSP_PWM_OUTPUT_COUNT) ? &g_pwm[index] : NULL;
}

bool BSP_PWM_SetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty duty)
{
    struct BSP_PWM_Resource *resource = pwm_from_handle(pwm);
    uint32_t compare;

    if ((resource == NULL) || (duty > BSP_PWM_DUTY_MAX)) {
        return false;
    }
    compare = (uint32_t)(((uint64_t)BSP_RESOURCE_MOTOR_PWM_PERIOD *
        (BSP_PWM_DUTY_MAX - duty)) / BSP_PWM_DUTY_MAX);
    DL_TimerA_setCaptureCompareValue(
        BSP_RESOURCE_MOTOR_PWM_TIMER, compare, resource->cc_index);
    resource->duty = duty;
    return true;
}

bool BSP_PWM_GetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty *duty)
{
    struct BSP_PWM_Resource *resource = pwm_from_handle(pwm);

    if ((resource == NULL) || (duty == NULL)) {
        return false;
    }
    *duty = resource->duty;
    return true;
}

bool BSP_PWM_GetFrequencyHz(BSP_PWM_Handle pwm, uint32_t *frequency_hz)
{
    if ((pwm_from_handle(pwm) == NULL) || (frequency_hz == NULL)) {
        return false;
    }
    *frequency_hz = BSP_PWM_FREQUENCY_HZ;
    return true;
}
