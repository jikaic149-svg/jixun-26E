/**
 * @file  MODE_CURVE.c
 * @brief 串级 PID 弧线行进
 *
 *   外环（角度）：(target_yaw - yaw)  → 目标角速度 Wz_target (°/s)
 *   内环（角速度）：(Wz_target - Wz)  → 差速补偿 ΔV (mm/s)
 *   输出：左轮 = base - ΔV，右轮 = base + ΔV
 *
 *   时序：内环 200Hz (5ms 控制环每周期跑一次)
 *         外环 50Hz  (每 4 个 inner tick 更新一次目标 Wz)
 *   阻尼来自内环本身的快速负反馈，不靠 D，避免对噪声敏感的角速度再求导
 */
#include "MODE_CURVE.h"
#include "../APP/broker/broker.h"
#include "../APP/algorithm/pid_controller.h"
#include "../APP/app_config.h"

#define OUTER_DIVIDER  4   /* 外环每 4 个 inner tick 更新一次 (5ms × 4 = 20ms = 50Hz) */

static PIDInstance s_pid_yaw_outer;
static PIDInstance s_pid_wz_inner;
static float       s_target_wz = 0.0f;
static float       s_last_diff = 0.0f;
static uint8_t     s_outer_cnt = 0;

void MODE_Curve_Init(void)
{
    PID_Init_Config_s outer_cfg = {
        .Kp            = PID_YAW_OUTER_KP,
        .Ki            = PID_YAW_OUTER_KI,
        .Kd            = 0.0f,
        .MaxOut        = PID_YAW_OUTER_MAX_WZ,
        .IntegralLimit = PID_YAW_OUTER_MAX_WZ,
        .DeadBand      = 0.0f,
        .Improve       = PID_Integral_Limit
    };
    PID_Init_Config_s inner_cfg = {
        .Kp            = PID_WZ_INNER_KP,
        .Ki            = PID_WZ_INNER_KI,
        .Kd            = 0.0f,                        /* 角速度本身已是噪声，禁用 D */
        .MaxOut        = PID_WZ_INNER_MAX_DIFF,
        .IntegralLimit = PID_WZ_INNER_INT_LIMIT,
        .DeadBand      = 0.0f,
        .Improve       = PID_Integral_Limit
    };

    PIDInit(&s_pid_yaw_outer, &outer_cfg);
    PIDInit(&s_pid_wz_inner,  &inner_cfg);
    s_target_wz = 0.0f;
    s_last_diff = 0.0f;
    s_outer_cnt = 0;
}

void MODE_Curve_Exec(float base_speed, float target_yaw, float current_yaw)
{
    /* === 外环：每 OUTER_DIVIDER 个 inner tick 更新一次目标 Wz === */
    if (++s_outer_cnt >= OUTER_DIVIDER) {
        s_outer_cnt = 0;

        float yaw_err = target_yaw - current_yaw;
        if (yaw_err >  180.0f) yaw_err -= 360.0f;
        if (yaw_err < -180.0f) yaw_err += 360.0f;

        /* PIDCalculate 用 (measure, target) 形式 → err = target - measure
         * 这里用 (0, yaw_err) 等价于直接对 yaw_err 做 P/I 运算 */
        s_target_wz = PIDCalculate(&s_pid_yaw_outer, 0.0f, yaw_err, 0.020f);
    }

    /* === 内环：每周期跑一次 Wz 闭环 === 
     * 极性必须取反！实锤了！
     * 日志中出现 TarL=100, TarR=500 且卡死，证明发生了正反馈死锁。
     * JY61P 的 Wz 极性与 Yaw 增量极性物理上是相反的，必须加负号纠正！
     */
    float wz   = -Broker_Subscribe_Float(TOPIC_JY61P_WZ);
    float diff = PIDCalculate(&s_pid_wz_inner, wz, s_target_wz, 0.005f);

    /* 速度柔和过渡：限制差速突变率 (Slew Rate Limiter)
     * 每周期 5ms，设最大允许步长 2.5f (相当于 500 mm/s^2 的差速加速度)
     * 这将彻底解决入弯瞬间轮速暴跳（一边 -20，一边 380）的粗暴感，变成温柔推入弯道 */
    const float MAX_DIFF_STEP = 3.5f; // 回调至3.5，配合晚切弯，迅速打满差速防止外抛 
    if (diff - s_last_diff > MAX_DIFF_STEP) {
        diff = s_last_diff + MAX_DIFF_STEP;
    } else if (diff - s_last_diff < -MAX_DIFF_STEP) {
        diff = s_last_diff - MAX_DIFF_STEP;
    }
    s_last_diff = diff;

    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT,  base_speed - diff);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, base_speed + diff);
}
