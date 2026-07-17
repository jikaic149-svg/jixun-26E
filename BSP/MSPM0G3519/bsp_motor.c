#include "bsp_motor.h"

#include <stddef.h>

#include "bsp_resource_map.h"

#define BSP_MOTOR_COUNT 2U

struct BSP_Motor_Resource {
    BSP_PWM_Handle pwm;
    uint32_t in1_pin;
    uint32_t in2_pin;
    BSP_Motor_State state;
};

static struct BSP_Motor_Resource g_motor[BSP_MOTOR_COUNT];

static struct BSP_Motor_Resource *motor_from_handle(BSP_Motor_Handle motor)
{
    for (size_t index = 0U; index < BSP_MOTOR_COUNT; ++index) {
        if (motor == &g_motor[index]) {
            return &g_motor[index];
        }
    }
    return NULL;
}

static void motor_set_direction(
    const struct BSP_Motor_Resource *motor, BSP_Motor_Mode mode)
{
    uint32_t set_mask = 0U;
    uint32_t clear_mask = motor->in1_pin | motor->in2_pin;

    if ((mode == BSP_MOTOR_MODE_FORWARD) ||
        (mode == BSP_MOTOR_MODE_BRAKE)) {
        set_mask |= motor->in1_pin;
    }
    if ((mode == BSP_MOTOR_MODE_REVERSE) ||
        (mode == BSP_MOTOR_MODE_BRAKE)) {
        set_mask |= motor->in2_pin;
    }
    DL_GPIO_clearPins(BSP_RESOURCE_MOTOR_GPIO_PORT, clear_mask & ~set_mask);
    DL_GPIO_setPins(BSP_RESOURCE_MOTOR_GPIO_PORT, set_mask);
}

void BSP_Motor_Init(void)
{
    g_motor[0] = (struct BSP_Motor_Resource){
        .pwm = BSP_PWM_GetHandle(0U),
        .in1_pin = BSP_RESOURCE_MOTOR_LEFT_IN1,
        .in2_pin = BSP_RESOURCE_MOTOR_LEFT_IN2,
        .state = {BSP_MOTOR_MODE_COAST, 0U},
    };
    g_motor[1] = (struct BSP_Motor_Resource){
        .pwm = BSP_PWM_GetHandle(1U),
        .in1_pin = BSP_RESOURCE_MOTOR_RIGHT_IN1,
        .in2_pin = BSP_RESOURCE_MOTOR_RIGHT_IN2,
        .state = {BSP_MOTOR_MODE_COAST, 0U},
    };
    motor_set_direction(&g_motor[0], BSP_MOTOR_MODE_COAST);
    motor_set_direction(&g_motor[1], BSP_MOTOR_MODE_COAST);
}

size_t BSP_Motor_Count(void)
{
    return BSP_MOTOR_COUNT;
}

BSP_Motor_Handle BSP_Motor_GetHandle(size_t index)
{
    return (index < BSP_MOTOR_COUNT) ? &g_motor[index] : NULL;
}

bool BSP_Motor_Apply(BSP_Motor_Handle motor, BSP_Motor_State state)
{
    struct BSP_Motor_Resource *resource = motor_from_handle(motor);

    if ((resource == NULL) || (state.mode > BSP_MOTOR_MODE_REVERSE) ||
        (state.duty > BSP_PWM_DUTY_MAX)) {
        return false;
    }
    if ((state.mode == BSP_MOTOR_MODE_COAST) ||
        (state.mode == BSP_MOTOR_MODE_BRAKE)) {
        state.duty = 0U;
    }

    if (!BSP_PWM_SetDuty(resource->pwm, 0U)) {
        return false;
    }
    motor_set_direction(resource, state.mode);
    if (!BSP_PWM_SetDuty(resource->pwm, state.duty)) {
        motor_set_direction(resource, BSP_MOTOR_MODE_COAST);
        return false;
    }
    resource->state = state;
    return true;
}

bool BSP_Motor_GetState(BSP_Motor_Handle motor, BSP_Motor_State *state)
{
    struct BSP_Motor_Resource *resource = motor_from_handle(motor);

    if ((resource == NULL) || (state == NULL)) {
        return false;
    }
    *state = resource->state;
    return true;
}
