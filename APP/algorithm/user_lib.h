/**
 * @file    user_lib.h
 * @brief   兼容性转发头 — 实现已统一到 math_utils.h/c
 * @note    此文件仅做 include 转发，并补充 user_lib 独有的 ARM/三角宏，
 *          所有函数实现在 math_utils.c 中，不会产生重定义错误。
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "math_utils.h"

/* 内存分配宏 — 可由用户覆盖 */
#ifndef user_malloc
#define user_malloc malloc
#endif

/* 三角函数快捷宏 */
#define msin(x) sinf(x)
#define mcos(x) cosf(x)

#endif /* _USER_LIB_H */
