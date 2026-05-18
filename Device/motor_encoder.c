/**
 * @file    motor_encoder.c
 * @brief   电机测速位置积分计算 (完全可移植纯软实现)
 */
#include "motor_encoder.h"

void MotorEncoder_Init(MotorEncoder_t *enc, MotorEncoder_Init_Config_t *config)
{
    if (!enc || !config) return;
    
    enc->GetPulseDiff = config->GetPulseDiff;
    enc->TotalResolution = config->TotalResolution;
    enc->WheelCircumference = config->WheelCircumference;
    
    MotorEncoder_Reset(enc);
}

void MotorEncoder_Update(MotorEncoder_t *enc, float dt_s)
{
    if (!enc) return;
    
    int16_t delta_p = 0;
    
    /* 1. 尝试从外部 BSP 读数回调取值，规避直接查 TIM 寄存器 */
    if (enc->GetPulseDiff) {
        delta_p = enc->GetPulseDiff();
    }
    
    /* 2. 累计最原始内部跳变脉冲 */
    enc->TotalPulses += delta_p;
    
    /* 3. 物理单位换算及瞬时一阶低通滤波 */
    if (dt_s > 0.0f && enc->TotalResolution > 0.0f) 
    {
        // 算出瞬时脉冲频率 (PPS) 及 20%一阶无限低通滤波
        float raw_pps = (float)delta_p / dt_s;
        enc->PulseRate = 0.5f * raw_pps + 0.5f * enc->PulseRate;
        
        // 线速度提取 (mm/s) = (转轴频率) * 周长
        enc->WheelSpeed = (enc->PulseRate / enc->TotalResolution) * enc->WheelCircumference;
        
        // 累计位移行程里程 (mm)
        enc->TotalDist = ((double)enc->TotalPulses / enc->TotalResolution) * enc->WheelCircumference;
    }
}

void MotorEncoder_Reset(MotorEncoder_t *enc)
{
    if (!enc) return;
    enc->TotalPulses = 0;
    enc->PulseRate = 0.0f;
    enc->WheelSpeed = 0.0f;
    enc->TotalDist = 0.0;
}
