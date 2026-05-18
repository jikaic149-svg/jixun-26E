/**
 * @file    math_utils.c
 * @brief   通用数学工具库实现
 * @origin  云台 modules/algorithm/user_lib.c
 * @note    100%可移植, 纯C实现
 */
#include "math_utils.h"

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr)
        memset(ptr, 0, size);
    return ptr;
}

float Sqrt(float x)
{
    float y, delta, maxError;
    if (x <= 0) return 0;
    y = x / 2;
    maxError = x * 0.001f;
    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);
    return y;
}

float abs_limit(float num, float Limit)
{
    if (num > Limit)       return Limit;
    else if (num < -Limit) return -Limit;
    return num;
}

float sign(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
        Value = 0.0f;
    return Value;
}

float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)      return minValue;
    else if (Value > maxValue) return maxValue;
    return Value;
}

int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)      return minValue;
    else if (Value > maxValue) return maxValue;
    return Value;
}

float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue) return Input;
    float len = maxValue - minValue;
    if (Input > maxValue)
    {
        while (Input > maxValue) Input -= len;
    }
    else if (Input < minValue)
    {
        while (Input < minValue) Input += len;
    }
    return Input;
}

float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    int integer = (int)raw;
    float decimal = raw - integer;
    if (decimal > 0.5f) integer++;
    return integer;
}

float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0.0f) { v[0] /= len; v[1] /= len; v[2] /= len; }
    return v;
}

float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData = (float *)zmalloc(row * col * sizeof(float));
}
