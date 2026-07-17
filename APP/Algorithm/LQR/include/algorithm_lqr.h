#ifndef ALGORITHM_LQR_H
#define ALGORITHM_LQR_H

#include <stdint.h>

#define ALGORITHM_LQR_MAX_STATE_DIM          6U
#define ALGORITHM_LQR_GAIN_SCHEDULE_MAX      4U
#define ALGORITHM_BALANCE_LQR_STATE_DIM      4U

typedef struct
{
    float position;
    float velocity;
    float theta;
    float theta_dot;
} Algorithm_BalanceLQR_State;

typedef struct
{
    uint8_t state_dim;
    float k[ALGORITHM_LQR_MAX_STATE_DIM];
    float u_min;
    float u_max;
} Algorithm_LQR_FixedGain;

typedef struct
{
    float schedule;
    float k[ALGORITHM_LQR_MAX_STATE_DIM];
} Algorithm_LQR_GainPoint;

typedef struct
{
    uint8_t state_dim;
    uint8_t point_count;
    Algorithm_LQR_GainPoint point[ALGORITHM_LQR_GAIN_SCHEDULE_MAX];
    float u_min;
    float u_max;
    float k_eff[ALGORITHM_LQR_MAX_STATE_DIM];
} Algorithm_LQR_GainSchedule;

typedef struct
{
    uint8_t state_dim;
    uint8_t ready;
    float k[ALGORITHM_LQR_MAX_STATE_DIM];
    float last_x[ALGORITHM_LQR_MAX_STATE_DIM];
    float u;
    float u_min;
    float u_max;
    float du_min;
    float du_max;
} Algorithm_LQR_Incremental;

void Algorithm_BalanceLQR_BuildState(float x[ALGORITHM_BALANCE_LQR_STATE_DIM],
                           float position,
                           float position_ref,
                           float velocity,
                           float velocity_ref,
                           float theta,
                           float theta_ref,
                           float theta_dot);
void Algorithm_BalanceLQR_MixYaw(float u_balance,
                       float yaw_cmd,
                       float u_min,
                       float u_max,
                       float *u_left,
                       float *u_right);

void Algorithm_LQR_FixedGain_Init(Algorithm_LQR_FixedGain *lqr,
                        const float *k,
                        uint8_t state_dim,
                        float u_min,
                        float u_max);
float Algorithm_LQR_FixedGain_Update(const Algorithm_LQR_FixedGain *lqr, const float *x);

void Algorithm_LQR_GainSchedule_Init(Algorithm_LQR_GainSchedule *lqr,
                           uint8_t state_dim,
                           float u_min,
                           float u_max);
uint8_t Algorithm_LQR_GainSchedule_SetPoint(Algorithm_LQR_GainSchedule *lqr,
                                  uint8_t index,
                                  float schedule,
                                  const float *k);
float Algorithm_LQR_GainSchedule_Update(Algorithm_LQR_GainSchedule *lqr,
                              float schedule,
                              const float *x);

void Algorithm_LQR_Incremental_Init(Algorithm_LQR_Incremental *lqr,
                          const float *k,
                          uint8_t state_dim,
                          float u_min,
                          float u_max,
                          float du_min,
                          float du_max);
void Algorithm_LQR_Incremental_Reset(Algorithm_LQR_Incremental *lqr,
                           float initial_u,
                           const float *x);
float Algorithm_LQR_Incremental_Update(Algorithm_LQR_Incremental *lqr, const float *x);

#endif
