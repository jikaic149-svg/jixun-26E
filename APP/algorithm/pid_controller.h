#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PID_None             = 0x00,
    PID_Integral_Limit   = 0x01,
    PID_Derivative_Filter = 0x02
} PID_Improvement_e;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;       /* 最大输出限幅 */
    float IntegralLimit;/* 积分限幅 */
    float DeadBand;     /* 死区 */
    uint32_t Improve;   /* 优化模式掩码 */
} PID_Init_Config_s;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    uint32_t Improve;

    float Measure;
    float Target;
    float Err;
    float Last_Err;
    float LL_Err;
    float Pout;
    float Iout;
    float Dout;
    float Output;
} PIDInstance;

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float measure, float target, float dt);

#endif /* PID_CONTROLLER_H */
