/**
 * @file    math_utils.h
 * @brief   通用数学工具库: 快速开方、限幅、死区、向量运算、均值滤波
 * @origin  云台 modules/algorithm/user_lib.h
 * @note    100%可移植. 已移除 ARM CMSIS DSP 依赖,
 *          如需使用矩阵运算, 可自行引入 arm_math.h 或用纯C矩阵实现
 */
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ======================== 常量定义 ======================== */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef PI
#define PI 3.14159265354f
#endif

/* ======================== 常用宏 ======================== */
#define VAL_LIMIT(val, min, max)  do { if ((val) <= (min)) (val) = (min); else if ((val) >= (max)) (val) = (max); } while(0)
#define VAL_MIN(a, b)  ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b)  ((a) > (b) ? (a) : (b))
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#define ANGLE_LIMIT_360(val, angle)  do { (val) = (angle) - (int)(angle); (val) += (int)(angle) % 360; } while(0)
#define ANGLE_LIMIT_360_TO_180(val)  do { if ((val) > 180) (val) -= 360; } while(0)

/* ======================== 简易矩阵类型(纯C, 替代arm_matrix_instance_f32) ======================== */
typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float   *pData;
} mat;

void MatInit(mat *m, uint8_t row, uint8_t col);

/* ======================== 函数声明 ======================== */
void  *zmalloc(size_t size);
float  Sqrt(float x);
float  abs_limit(float num, float Limit);
float  sign(float value);
float  float_deadband(float Value, float minValue, float maxValue);
float  float_constrain(float Value, float minValue, float maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
float  loop_float_constrain(float Input, float minValue, float maxValue);
float  theta_format(float Ang);
int    float_rounding(float raw);
float *Norm3d(float *v);
float  NormOf3d(float *v);
void   Cross3d(float *v1, float *v2, float *res);
float  Dot3d(float *v1, float *v2);
float  AverageFilter(float new_data, float *buf, uint8_t len);

#endif /* MATH_UTILS_H */
