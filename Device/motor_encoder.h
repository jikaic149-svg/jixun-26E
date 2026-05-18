/**
 * @file    motor_encoder.h
 * @brief   OOP 架构电机通用测距/测速编码器分析模块
 * @note    剥离 F407 强耦合，输入脉冲差与 dt 计算出纯物理速度
 */
#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

/**
 * @brief 提供硬件当前循环中编码器（如由 TIM 读取后的插值）的回调
 */
typedef int16_t (*MotorEncoder_GetPulseDiff_Func_t)(void);

/**
 * @brief 电机编码器物理量解算器实例
 */
typedef struct {
    MotorEncoder_GetPulseDiff_Func_t GetPulseDiff;
    
    /* 物理常数 */
    float    TotalResolution;     // 每圈综合脉冲 (线数 * 倍频 * 减速比)
    float    WheelCircumference;  // 轮周长 (mm)
    
    /* 运行时状态 */
    int64_t  TotalPulses;         // 累计微元脉冲数
    float    PulseRate;           // 低通滤波后的脉冲频率 (pps)
    
    /* 对外物理量输出 (外部控制可读) */
    float    WheelSpeed;          // 实测线速度 (mm/s)
    double   TotalDist;           // 累计行程里程 (mm)
} MotorEncoder_t;

/**
 * @brief 初始化配置载体结构
 */
typedef struct {
    MotorEncoder_GetPulseDiff_Func_t GetPulseDiff;
    float    TotalResolution;
    float    WheelCircumference;
} MotorEncoder_Init_Config_t;

/* ======= API声明 ======= */
void MotorEncoder_Init(MotorEncoder_t *enc, MotorEncoder_Init_Config_t *config);
void MotorEncoder_Update(MotorEncoder_t *enc, float dt_s);
void MotorEncoder_Reset(MotorEncoder_t *enc);

#endif /* MOTOR_ENCODER_H */
