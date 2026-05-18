/**
 * @file inertial_nav.c
 * @brief 惯性导航解算应用模块 (属于 Application 层)
 *        优化：滑动窗口均值滤波 + ZUPT 零速修正 + 速度指数衰减
 */

#include "inertial_nav.h"
#include <math.h>

/* ======================== 可调参数 ======================== */
#define ACC_DEADZONE      0.08f   /* 零飘死区阈值 (m/s^2)                  */
#define ZUPT_THRESHOLD    0.05f   /* ZUPT 零速判定加速度阈值 (m/s^2)       */
#define ZUPT_HOLD_COUNT   5       /* 连续 N 个周期在阈值内 → 判定静止       */
                                  /* 5 × 10ms = 50ms 即可触发归零           */
#define SPEED_DECAY       0.95f   /* 每周期速度衰减系数，防止噪声长期累积    */
#define ZUPT_SPEED_THRESH 30.0f   /* ZUPT 速度门限 (mm/s)，速度低于此值才允许归零 */

/* ---- 转弯里程补偿参数 ---- */
#define PIVOT_TURN_THRESHOLD  60.0f  /* 角速度 > 此值 → 原地转弯，冻结里程   */
#define CURVE_TURN_THRESHOLD  10.0f  /* 角速度 > 此值 → 行进间转弯，里程打折  */
#define CURVE_MILEAGE_SCALE   0.7f   /* 行进间转弯里程缩放系数               */
/* ========================================================= */

void INS_Init(INS_Instance *inst)
{
    inst->speed_x       = 0.0f;
    inst->mileage_x     = 0.0f;
    inst->ax_offset     = 0.0f;
    inst->calib_count   = 0;
    inst->zupt_counter  = 0;
    inst->filter_idx    = 0;
    inst->filter_filled = 0;
    for (uint8_t i = 0; i < INS_AX_FILTER_SIZE; i++) {
        inst->ax_filter_buf[i] = 0.0f;
    }
}

void INS_UpdateSpeedMileage(INS_Instance *inst, float ax_mps2, float yaw_rate, float dt_s)
{
    /* ============================================================
     * 1. 开机前 100 个周期 (~1s) 静态零偏自校准
     *    前提：开机这 1 秒内小车必须保持绝对静止
     * ============================================================ */
    if (inst->calib_count < 100) {
        inst->ax_offset += ax_mps2;
        inst->calib_count++;
        if (inst->calib_count == 100) {
            inst->ax_offset /= 100.0f;
        }
        return; /* 校准期间不输出速度和里程 */
    }

    /* ============================================================
     * 2. 减去系统偏置
     * ============================================================ */
    ax_mps2 -= inst->ax_offset;

    /* ============================================================
     * 3. 滑动窗口均值滤波 — 抑制高频噪声突破死区
     * ============================================================ */
    inst->ax_filter_buf[inst->filter_idx] = ax_mps2;
    inst->filter_idx = (inst->filter_idx + 1) % INS_AX_FILTER_SIZE;
    if (!inst->filter_filled && inst->filter_idx == 0) {
        inst->filter_filled = 1;
    }

    uint8_t count = inst->filter_filled ? INS_AX_FILTER_SIZE : inst->filter_idx;
    if (count == 0) count = 1; /* 安全兜底 */
    float ax_sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        ax_sum += inst->ax_filter_buf[i];
    }
    ax_mps2 = ax_sum / (float)count;

    /* ============================================================
     * 4. 死区滤波 — 过滤掉偏置补偿后的剩余白噪声底噪
     * ============================================================ */
    if (ax_mps2 > -ACC_DEADZONE && ax_mps2 < ACC_DEADZONE) {
        ax_mps2 = 0.0f;
    }

    /* ============================================================
     * 5. ZUPT 零速修正
     *    必须同时满足：加速度接近零 AND 当前速度接近零
     *    防止匀速行驶时（加速度≈0但速度不为0）被误判为静止
     * ============================================================ */
    if (fabsf(ax_mps2) < ZUPT_THRESHOLD) {
        inst->zupt_counter++;
    } else {
        inst->zupt_counter = 0;
    }

    if (inst->zupt_counter >= ZUPT_HOLD_COUNT &&
        fabsf(inst->speed_x) < ZUPT_SPEED_THRESH) {
        inst->speed_x = 0.0f;   /* 核心：强制归零！ */
        inst->zupt_counter = ZUPT_HOLD_COUNT; /* 防溢出 */
        return; /* 静止时不再积分里程 */
    }

    /* ============================================================
     * 6. 欧拉积分解算速度  v = v0 + a * dt
     *    物理换算: m/s^2 × 1000 = mm/s^2
     * ============================================================ */
    inst->speed_x += (ax_mps2 * 1000.0f) * dt_s;

    /* ============================================================
     * 7. 速度指数衰减 — 模拟摩擦损耗，防止噪声长时间累积
     * ============================================================ */
    inst->speed_x *= SPEED_DECAY;

    /* ============================================================
     * 7.5 非负钳位 — 不允许倒车，速度 < 0 直接归零
     * ============================================================ */
    if (inst->speed_x < 0.0f) {
        inst->speed_x = 0.0f;
    }

    /* ============================================================
     * 8. 转弯状态判定 + 里程积分
     *    根据 yaw_rate 区分：直线 / 行进间转弯 / 原地转弯
     * ============================================================ */
    float abs_yaw_rate = fabsf(yaw_rate);

    if (abs_yaw_rate > PIVOT_TURN_THRESHOLD) {
        /* 原地转弯：车身在旋转但不前进 → 冻结里程 */
        /* 不累加 mileage_x */
    } else if (abs_yaw_rate > CURVE_TURN_THRESHOLD) {
        /* 行进间转弯：向心加速度导致速度偏大 → 里程打折 */
        inst->mileage_x += inst->speed_x * dt_s * CURVE_MILEAGE_SCALE;
    } else {
        /* 直线行驶：正常积分 */
        inst->mileage_x += inst->speed_x * dt_s;
    }
}

