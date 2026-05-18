/**
 * @file    quaternion_ekf.h
 * @brief   四元数扩展卡尔曼滤波(EKF)姿态解算, 含陀螺仪零偏估计和卡方检验
 * @author  Wang Hongxi
 * @origin  云台 modules/algorithm/QuaternionEKF.h
 * @note    依赖 kalman_filter.h 提供的矩阵运算
 */
#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#include "kalman_filter.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];         // 四元数估计值
    float GyroBias[3];  // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1;   // 四元数更新过程噪声
    float Q2;   // 陀螺仪零偏过程噪声
    float R;    // 加速度计量测噪声

    float dt;
    mat ChiSquare;
    float ChiSquare_Data[1];
    float ChiSquareTestThreshold;
    float lambda;   // 渐消因子

    int16_t YawRoundCount;
    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

/**
 * @brief 初始化四元数EKF
 * @param init_quaternion  初始四元数[4] (通常为{1,0,0,0})
 * @param process_noise1   四元数过程噪声 (推荐10)
 * @param process_noise2   陀螺仪零偏过程噪声 (推荐0.001)
 * @param measure_noise    加速度计量测噪声 (推荐1000000)
 * @param lambda           渐消系数 (推荐0.9996)
 * @param lpf              低通滤波系数 (推荐0)
 */
void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1,
                            float process_noise2, float measure_noise,
                            float lambda, float lpf);

/**
 * @brief 四元数EKF更新
 * @param gx,gy,gz  陀螺仪数据(rad/s)
 * @param ax,ay,az  加速度计数据(m/s²)
 * @param dt        更新周期(s)
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt);

#endif /* QUATERNION_EKF_H */
