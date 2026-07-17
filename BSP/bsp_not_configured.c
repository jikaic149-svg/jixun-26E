#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_line_sensor.h"
#include "bsp_motor.h"
#include "bsp_pwm.h"
#include "bsp_resource_map.h"
#include "bsp_servo.h"
#include "bsp_solenoid.h"
#include "bsp_uart.h"

/* Safe zero-resource backends for peripherals that have not been assigned in
 * this MCU's SysConfig/resource map. A concrete BSP replaces only the module
 * it maps; no pin, timer, UART, or DMA channel is guessed here. */
#if BSP_RESOURCE_HAS_CAN == 0
size_t BSP_CAN_Count(void)
{
    return 0U;
}

BSP_CAN_Handle BSP_CAN_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_CAN_GetInfo(BSP_CAN_Handle can, BSP_CAN_Info *info)
{
    (void)can;
    (void)info;
    return false;
}

BSP_CAN_Result BSP_CAN_Send(
    BSP_CAN_Handle can, const BSP_CAN_Frame *frame, uint32_t timeout_ms)
{
    (void)can;
    (void)frame;
    (void)timeout_ms;
    return BSP_CAN_RESULT_UNSUPPORTED;
}

BSP_CAN_Result BSP_CAN_ReadAvailable(
    BSP_CAN_Handle can, BSP_CAN_Frame *frame)
{
    (void)can;
    (void)frame;
    return BSP_CAN_RESULT_UNSUPPORTED;
}

bool BSP_CAN_IsBusOff(BSP_CAN_Handle can)
{
    (void)can;
    return false;
}

BSP_CAN_Result BSP_CAN_Recover(BSP_CAN_Handle can)
{
    (void)can;
    return BSP_CAN_RESULT_UNSUPPORTED;
}
#endif

#if BSP_RESOURCE_HAS_SOLENOID == 0
size_t BSP_Solenoid_Count(void)
{
    return 0U;
}

bool BSP_Solenoid_Set(size_t index, bool on)
{
    (void)index;
    (void)on;
    return false;
}

bool BSP_Solenoid_Get(size_t index, bool *on)
{
    (void)index;
    (void)on;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_SERVO == 0
size_t BSP_Servo_Count(void)
{
    return 0U;
}

BSP_Servo_Handle BSP_Servo_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_Servo_SetEnabled(BSP_Servo_Handle servo, bool enabled)
{
    (void)servo;
    (void)enabled;
    return false;
}

bool BSP_Servo_IsEnabled(BSP_Servo_Handle servo, bool *enabled)
{
    (void)servo;
    (void)enabled;
    return false;
}

bool BSP_Servo_SetPulseUs(BSP_Servo_Handle servo, uint32_t pulse_us)
{
    (void)servo;
    (void)pulse_us;
    return false;
}

bool BSP_Servo_GetPulseUs(BSP_Servo_Handle servo, uint32_t *pulse_us)
{
    (void)servo;
    (void)pulse_us;
    return false;
}

bool BSP_Servo_GetFramePeriodUs(
    BSP_Servo_Handle servo, uint32_t *period_us)
{
    (void)servo;
    (void)period_us;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_KEY == 0
size_t BSP_Key_Count(void)
{
    return 0U;
}

BSP_Key_Handle BSP_Key_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_Key_Read(BSP_Key_Handle key, bool *pressed)
{
    (void)key;
    (void)pressed;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_LINE_SENSOR == 0
size_t BSP_LineSensor_Count(void)
{
    return 0U;
}

BSP_LineSensor_Handle BSP_LineSensor_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_LineSensor_GetInfo(
    BSP_LineSensor_Handle sensor, BSP_LineSensor_Info *info)
{
    (void)sensor;
    (void)info;
    return false;
}

bool BSP_LineSensor_Read(
    BSP_LineSensor_Handle sensor, BSP_LineSensor_RawSample *sample)
{
    (void)sensor;
    (void)sample;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_MOTOR == 0
size_t BSP_Motor_Count(void)
{
    return 0U;
}

BSP_Motor_Handle BSP_Motor_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_Motor_Apply(BSP_Motor_Handle motor, BSP_Motor_State state)
{
    (void)motor;
    (void)state;
    return false;
}

bool BSP_Motor_GetState(BSP_Motor_Handle motor, BSP_Motor_State *state)
{
    (void)motor;
    (void)state;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_UART == 0
size_t BSP_UART_Count(void)
{
    return 0U;
}

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    (void)uart;
    (void)info;
    return false;
}

BSP_UART_Result BSP_UART_WriteBlocking(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    if (transferred != NULL) {
        *transferred = 0U;
    }
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length)
{
    (void)uart;
    (void)data;
    (void)length;
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

BSP_UART_Result BSP_UART_ReadBlocking(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    if (transferred != NULL) {
        *transferred = 0U;
    }
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

BSP_UART_Result BSP_UART_ReadAvailable(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t capacity,
    size_t *transferred)
{
    (void)uart;
    (void)data;
    (void)capacity;
    if (transferred != NULL) {
        *transferred = 0U;
    }
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

bool BSP_UART_GetRxStatus(BSP_UART_Handle uart, BSP_UART_RxStatus *status)
{
    (void)uart;
    (void)status;
    return false;
}

bool BSP_UART_ClearRxStatus(BSP_UART_Handle uart)
{
    (void)uart;
    return false;
}

bool BSP_UART_Recover(BSP_UART_Handle uart)
{
    (void)uart;
    return false;
}

BSP_UART_Result BSP_UART_StartWriteDMA(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

BSP_UART_Result BSP_UART_StartReadDMA(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return BSP_UART_RESULT_INVALID_ARGUMENT;
}

bool BSP_UART_IsTxBusy(BSP_UART_Handle uart)
{
    (void)uart;
    return false;
}

bool BSP_UART_IsRxBusy(BSP_UART_Handle uart)
{
    (void)uart;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_PWM == 0
size_t BSP_PWM_Count(void)
{
    return 0U;
}

BSP_PWM_Handle BSP_PWM_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_PWM_SetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty duty)
{
    (void)pwm;
    (void)duty;
    return false;
}

bool BSP_PWM_GetDuty(BSP_PWM_Handle pwm, BSP_PWM_Duty *duty)
{
    (void)pwm;
    (void)duty;
    return false;
}

bool BSP_PWM_GetFrequencyHz(BSP_PWM_Handle pwm, uint32_t *frequency_hz)
{
    (void)pwm;
    (void)frequency_hz;
    return false;
}
#endif

#if BSP_RESOURCE_HAS_ENCODER == 0
size_t BSP_Encoder_Count(void)
{
    return 0U;
}

BSP_Encoder_Handle BSP_Encoder_GetHandle(size_t index)
{
    (void)index;
    return NULL;
}

bool BSP_Encoder_GetSnapshot(
    BSP_Encoder_Handle encoder, int64_t *cumulative_count)
{
    (void)encoder;
    (void)cumulative_count;
    return false;
}

bool BSP_Encoder_ReadDelta(BSP_Encoder_Handle encoder, int64_t *delta_count)
{
    (void)encoder;
    (void)delta_count;
    return false;
}
#endif
