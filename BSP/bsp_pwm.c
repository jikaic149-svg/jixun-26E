#include "bsp_pwm.h"
#include "ti_msp_dl_config.h"

void BSP_PWM_Init(void) {
    DL_TimerA_startCounter(PWM_0_INST);
}

void BSP_PWM_SetDutyL(uint32_t duty) {
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, duty, GPIO_PWM_0_C0_IDX);
}

void BSP_PWM_SetDutyR(uint32_t duty) {
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, duty, GPIO_PWM_0_C1_IDX);
}
