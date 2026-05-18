/**
 * @file    kalman_filter.h
 * @brief   通用卡尔曼滤波器框架（支持自定义维度、自动调整、用户扩展函数）
 * @author  Wang Hongxi
 * @origin  云台 modules/algorithm/kalman_filter.h
 * @note    需要矩阵运算支持, 可使用 ARM CMSIS DSP 或自行实现以下矩阵操作:
 *          Matrix_Init, Matrix_Add, Matrix_Subtract, Matrix_Multiply,
 *          Matrix_Transpose, Matrix_Inverse
 *          如在非ARM平台使用, 需要提供这些函数的纯C实现
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ======================== 矩阵运算接口 ======================== */
/*
 * 如果使用 ARM CMSIS DSP, 取消下面的注释:
 * #include "arm_math.h"
 * #define mat arm_matrix_instance_f32
 * #define Matrix_Init      arm_mat_init_f32
 * #define Matrix_Add       arm_mat_add_f32
 * #define Matrix_Subtract  arm_mat_sub_f32
 * #define Matrix_Multiply  arm_mat_mult_f32
 * #define Matrix_Transpose arm_mat_trans_f32
 * #define Matrix_Inverse   arm_mat_inverse_f32
 *
 * 如果不用 ARM, 需自行提供 mat 类型和上述函数
 */

#ifndef KALMAN_MAT_DEFINED
#define KALMAN_MAT_DEFINED
/* 简易矩阵类型, 兼容 arm_matrix_instance_f32 布局 */
typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float   *pData;
} KF_Mat_t;
#define mat KF_Mat_t

/* 用户需实现或链接以下函数 */
extern void  Matrix_Init(mat *m, uint16_t rows, uint16_t cols, float *data);
extern int32_t Matrix_Add(const mat *A, const mat *B, mat *C);
extern int32_t Matrix_Subtract(const mat *A, const mat *B, mat *C);
extern int32_t Matrix_Multiply(const mat *A, const mat *B, mat *C);
extern int32_t Matrix_Transpose(const mat *A, mat *C);
extern int32_t Matrix_Inverse(const mat *A, mat *C);
#endif

#ifndef user_malloc
#define user_malloc malloc
#endif

/* ======================== 卡尔曼滤波器结构体 ======================== */
typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;       // 状态维度
    uint8_t uSize;          // 控制输入维度
    uint8_t zSize;          // 量测维度

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;
    uint8_t *temp;

    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;       // x(k|k)
    mat xhatminus;  // x(k|k-1)
    mat u;          // 控制向量
    mat z;          // 量测向量
    mat P;          // 协方差矩阵 P(k|k)
    mat Pminus;     // 先验协方差 P(k|k-1)
    mat F, FT;      // 状态转移矩阵
    mat B;          // 控制矩阵
    mat H, HT;      // 量测矩阵
    mat Q;          // 过程噪声协方差
    mat R;          // 量测噪声协方差
    mat K;          // 卡尔曼增益
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    /* 用户自定义扩展函数 */
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    /* 矩阵存储空间 */
    float *xhat_data, *xhatminus_data;
    float *u_data, *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data, *R_data, *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1;
    float *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

/* ======================== 函数声明 ======================== */
void   Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void   Kalman_Filter_Measure(KalmanFilter_t *kf);
void   Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void   Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void   Kalman_Filter_SetK(KalmanFilter_t *kf);
void   Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void   Kalman_Filter_P_Update(KalmanFilter_t *kf);
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif /* KALMAN_FILTER_H */
