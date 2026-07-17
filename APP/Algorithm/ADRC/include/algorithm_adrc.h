#ifndef ALGORITHM_ADRC_H
#define ALGORITHM_ADRC_H

#include <stdint.h>

typedef struct
{
    float b0;
    float wo;
    float beta1;
    float beta2;
    float z1;
    float z2;
    float z2_limit;
    uint8_t ready;
} Algorithm_ADRC_FirstOrderESO;

typedef struct
{
    Algorithm_ADRC_FirstOrderESO eso;
    float kp;
    float ki;
    float kaw;
    float u_max;
    float integral;
    float last_u;
    float risk;
    uint8_t integral_enable;
    uint8_t compensation_enable;
} Algorithm_ADRC_PI_ESO;

void Algorithm_ADRC_FirstOrderESO_Init(Algorithm_ADRC_FirstOrderESO *eso,
                         float b0,
                         float wo,
                         float z2_limit);
void Algorithm_ADRC_FirstOrderESO_Reset(Algorithm_ADRC_FirstOrderESO *eso, float omega);
void Algorithm_ADRC_FirstOrderESO_SetParams(Algorithm_ADRC_FirstOrderESO *eso,
                              float b0,
                              float wo,
                              float z2_limit);
void Algorithm_ADRC_FirstOrderESO_Update(Algorithm_ADRC_FirstOrderESO *eso,
                           float omega,
                           float u_last,
                           float dt_sec);
float Algorithm_ADRC_FirstOrderESO_GetOmega(const Algorithm_ADRC_FirstOrderESO *eso);
float Algorithm_ADRC_FirstOrderESO_GetDisturbance(const Algorithm_ADRC_FirstOrderESO *eso);
float Algorithm_ADRC_FirstOrderESO_GetLimitedDisturbance(const Algorithm_ADRC_FirstOrderESO *eso);
float Algorithm_ADRC_FirstOrderESO_Compensate(const Algorithm_ADRC_FirstOrderESO *eso,
                                float u_base,
                                float u_max);

void Algorithm_ADRC_PI_ESO_Init(Algorithm_ADRC_PI_ESO *ctrl,
                 float kp,
                 float ki,
                 float kaw,
                 float b0,
                 float wo,
                 float u_max,
                 float z2_limit);
void Algorithm_ADRC_PI_ESO_Reset(Algorithm_ADRC_PI_ESO *ctrl, float omega);
void Algorithm_ADRC_PI_ESO_SetGains(Algorithm_ADRC_PI_ESO *ctrl,
                     float kp,
                     float ki,
                     float kaw);
void Algorithm_ADRC_PI_ESO_EnableIntegral(Algorithm_ADRC_PI_ESO *ctrl, uint8_t enable);
void Algorithm_ADRC_PI_ESO_EnableCompensation(Algorithm_ADRC_PI_ESO *ctrl, uint8_t enable);
float Algorithm_ADRC_PI_ESO_CalcRisk(float theta_deg,
                      float theta_dot_dps,
                      float theta_limit_deg,
                      float theta_dot_limit_dps);
float Algorithm_ADRC_PI_ESO_Update(Algorithm_ADRC_PI_ESO *ctrl,
                    float omega_ref,
                    float omega,
                    float theta_deg,
                    float theta_dot_dps,
                    float theta_limit_deg,
                    float theta_dot_limit_dps,
                    float dt_sec);

#endif
