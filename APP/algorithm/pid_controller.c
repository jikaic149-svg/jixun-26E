#include "pid_controller.h"

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) {
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->IntegralLimit = config->IntegralLimit;
    pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve;

    pid->Measure = 0;
    pid->Target = 0;
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->LL_Err = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->Output = 0;
}

float PIDCalculate(PIDInstance *pid, float measure, float target, float dt) {
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = target - measure;
    
    if (pid->Err > -pid->DeadBand && pid->Err < pid->DeadBand) {
        pid->Err = 0;
    }
    
    pid->Pout = pid->Kp * pid->Err;
    pid->Iout += pid->Ki * pid->Err * dt;
    /* 简化的微分项，避免突变 */
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / dt;
    
    /* 积分抗饱和保护 */
    if (pid->Improve & PID_Integral_Limit) {
        if (pid->Iout > pid->IntegralLimit) pid->Iout = pid->IntegralLimit;
        if (pid->Iout < -pid->IntegralLimit) pid->Iout = -pid->IntegralLimit;
    }
    
    pid->Output = pid->Pout + pid->Iout + pid->Dout;
    
    /* 输出总限幅 */
    if (pid->Output > pid->MaxOut) pid->Output = pid->MaxOut;
    if (pid->Output < -pid->MaxOut) pid->Output = -pid->MaxOut;
    
    pid->Last_Err = pid->Err;
    
    return pid->Output;
}
