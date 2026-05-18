/**
 * @file    attitude_algorithm.c
 * @brief   Mahony互补滤波姿态解算 + 二阶Biquad滤波器
 * @origin  底盘 project/code/icm.c
 * @note    100%可移植, 纯数学算法
 *          已移除对硬件驱动的直接依赖
 *          用户需自行读取IMU数据后调用 MahonyAHRSupdateIMU()
 */
#include "attitude_algorithm.h"

/* ======================== Mahony滤波器参数 ======================== */
#define sampleFreq  200.0f      // 采样频率(Hz), 根据实际修改
#define twoKpDef    (2.0f * 0.5f)   // 比例增益, 越高响应越快但可能超调
#define twoKiDef    (2.0f * 0.0f)   // 积分增益, 用于长期零偏校正

static volatile float invsampleFreq = 1.0f / sampleFreq;
static volatile float twoKp = twoKpDef;
static volatile float twoKi = twoKiDef;
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/* ======================== 姿态角全局变量 ======================== */
float Pitch_a, Roll_a, Yaw_a;
static float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;
float Pitch_g, Roll_g, Yaw_g;
float Pitch_g_F, Roll_g_F, Yaw_g_F;
float Yaw_TotalAngle = 0.0f;
float Yaw_AngleLast = 0.0f;
int Yaw_RoundCount = 0;

/* 滤波器实例 */
static biquad_state Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad;
static biquad_state Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;
biquad_state adc_error;

/* ======================== 滤波器初始化 ======================== */
void Filter_Init(void)
{
    // 角速度滤波器
    biquad_filter_init(&Pitch_g_biquad,   BIQUAD_LOWPASS, 200, 30, 0.7071f);
    biquad_filter_init(&Roll_g_biquad,    BIQUAD_LOWPASS, 200, 20, 0.5773f);
    biquad_filter_init(&Yaw_g_biquad,     BIQUAD_LOWPASS, 200, 30, 0.5773f);
    // 加速度滤波器
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.5773f);
    biquad_filter_init(&Roll_acc_biquad,  BIQUAD_LOWPASS, 200, 10, 0.5773f);
    biquad_filter_init(&Yaw_acc_biquad,   BIQUAD_LOWPASS, 200, 10, 0.5773f);
    // 预留
    biquad_filter_init(&adc_error,        BIQUAD_LOWPASS, 200, 10, 0.5773f);
}

/* ======================== Mahony互补滤波 ======================== */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 角速度转弧度
        gx *= 0.0174532925f;
        gy *= 0.0174532925f;
        gz *= 0.0174532925f;

        // 归一化加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 交叉积误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分反馈
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invsampleFreq;
            integralFBy += twoKi * halfey * invsampleFreq;
            integralFBz += twoKi * halfez * invsampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数微分方程积分
    gx *= (0.5f * invsampleFreq);
    gy *= (0.5f * invsampleFreq);
    gz *= (0.5f * invsampleFreq);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 旋转矩阵 -> 欧拉角
    float r11 = 2.0f * (q0 * q1 + q2 * q3);
    float r12 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float r21 = 2.0f * (q0 * q2 - q3 * q1);
    float r31 = 2.0f * (q0 * q3 + q1 * q2);
    float r32 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    Yaw_a_Pi   = atan2f(r31, r32);
    Pitch_a_Pi = -asinf(r21);
    Roll_a_Pi  = atan2f(r11, r12);

    Pitch_a = -Rad2Ang * Pitch_a_Pi;
    Roll_a  =  Rad2Ang * Roll_a_Pi;
    Yaw_a   =  Rad2Ang * Yaw_a_Pi;

    // 多圈Yaw角处理
    if (Yaw_a - Yaw_AngleLast > 180.0f)
        Yaw_RoundCount--;
    else if (Yaw_a - Yaw_AngleLast < -180.0f)
        Yaw_RoundCount++;
    Yaw_TotalAngle = 360.0f * Yaw_RoundCount + Yaw_a;
    Yaw_AngleLast = Yaw_a;
}

/* ======================== 二阶Biquad滤波器 ======================== */
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)// fs: 采样频率, fc: 截止频率, q_value: 品质因数
{
    float w0 = 2.0f * PI * fc / fs;
    float sin_w0 = sinf(w0);
    float cos_w0 = cosf(w0);
    float alpha = sin_w0 / (2.0f * q_value);
    float b0, b1, b2, a0, a1, a2;

    switch (type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0f - cos_w0) / 2.0f;
        b1 = b0 * 2;
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0f + cos_w0) / 2.0f;
        b1 = -b0 * 2;
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cos_w0;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    default:
        return;
    }

    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0f;
    state->y1 = state->y2 = 0.0f;
}

float biquad(biquad_state *state, float data)
{
    float result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2
                 - state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;
    return result;
}

/* ======================== 快速开方倒数 ======================== */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
