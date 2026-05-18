/**
 * @file  mode_manager.c
 * @brief 小车行为状态机管理器实现
 *        纯软逻辑，不含任何 MCU 头文件，所有数据交互走 Broker
 */
#include "mode_manager.h"
#include "MODE_IDLE.h"
#include "MODE_STRAIGHT.h"
//#include "MODE_TRACKING.h"
#include "MODE_TURNING.h"
#include "MODE_CURVE.h"
#include "MODE_BRAKE.h"
#include "MODE_DONE.h"
#include "../Task/MODE_TASK_TEST.h"
#include "../APP/broker/broker.h"
#include "../Device/device_buzzer.h"

/* ======================== 模式上下文 ======================== */
typedef struct {
    CarMode_e mode;
    float base_pwm;
    float target_yaw;
    float max_pwm;
} ModeContext_t;

static ModeContext_t s_ctx = {
    .mode       = CAR_MODE_IDLE,
    .base_pwm   = 0.0f,
    .target_yaw = 0.0f,
    .max_pwm    = 0.0f
};

/* ======================== 公共 API ======================== */

static uint8_t s_task_running = 0;

void Mode_Init(void)
{
    s_ctx.mode = CAR_MODE_IDLE;
    s_task_running = 0;
    MODE_Idle_Init();
    Device_Buzzer_Set(0); // 防止低电平触发蜂鸣器一上电就狂叫
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_IDLE);
}

void Mode_Exec(void)
{
    // 如果测试流水线在运行，优先让它执行逻辑跳转（它会调用 Mode_Setxxx）
    if (s_task_running) {
        TaskTest_Exec();
    }

    float yaw   = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
    float track = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);

    switch (s_ctx.mode)
    {
        case CAR_MODE_IDLE:
            MODE_Idle_Exec();
            break;

        case CAR_MODE_STRAIGHT:
            MODE_Straight_Exec(s_ctx.base_pwm, s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_TRACKING:
            // MODE_Tracking_Exec(s_ctx.max_pwm, track);
            break;

        case CAR_MODE_TURNING:
            MODE_Turning_Exec(s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_CURVE:
            MODE_Curve_Exec(s_ctx.base_pwm, s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_BRAKE:
            MODE_Brake_Exec();
            break;

        case CAR_MODE_DONE:
            MODE_Done_Exec();
            break;

        case CAR_MODE_TASK_TEST:
            // 兼容以前的残留
            TaskTest_Exec();
            break;

        default:
            MODE_Idle_Exec();
            break;
    }
}

CarMode_e Mode_GetMode(void)
{
    return s_ctx.mode;
}

/* ======================== 模式切换 ======================== */

void Mode_SetIdle(void)
{
    s_task_running = 0;
    s_ctx.mode = CAR_MODE_IDLE;
    MODE_Idle_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_IDLE);
}

void Mode_SetStraight(float base_pwm, float target_yaw)
{
    if (s_ctx.mode != CAR_MODE_STRAIGHT) {
        MODE_Straight_Init();
    }
    s_ctx.mode       = CAR_MODE_STRAIGHT;
    s_ctx.base_pwm   = base_pwm;
    s_ctx.target_yaw = target_yaw;
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_STRAIGHT);
}

void Mode_SetTracking(float max_pwm)
{
    if (s_ctx.mode != CAR_MODE_TRACKING) {
        // MODE_Tracking_Init();
    }
    s_ctx.mode    = CAR_MODE_TRACKING;
    s_ctx.max_pwm = max_pwm;
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TRACKING);
}

void Mode_SetTurning(float target_yaw)
{
    if (s_ctx.mode != CAR_MODE_TURNING) {
        MODE_Turning_Init();
    }
    s_ctx.mode       = CAR_MODE_TURNING;
    s_ctx.target_yaw = target_yaw;
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TURNING);
}

void Mode_SetCurve(float base_pwm, float target_yaw)
{
    if (s_ctx.mode != CAR_MODE_CURVE) {
        MODE_Curve_Init();
    }
    s_ctx.mode       = CAR_MODE_CURVE;
    s_ctx.base_pwm   = base_pwm;
    s_ctx.target_yaw = target_yaw;
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_CURVE);
}

void Mode_SetBrake(void)
{
    s_ctx.mode = CAR_MODE_BRAKE;
    MODE_Brake_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_BRAKE);
}

void Mode_SetDone(void)
{
    s_task_running = 0;
    s_ctx.mode = CAR_MODE_DONE;
    MODE_Done_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_DONE);
}

void Mode_SetTask(uint8_t task_id)
{
    /* 唯一任务：PID基础测试流水线（task_id 仅作占位语义） */
    (void)task_id;
    s_task_running = 1;
    // 不再把 s_ctx.mode 设置为 CAR_MODE_TASK_TEST，因为任务本身会调度底层模式
    TaskTest_Init();
    Broker_Publish_Int(TOPIC_TASK_ID, 1);
}

/* ======================== 查询接口 ======================== */

uint8_t Mode_IsTurningDone(void)
{
    if (s_ctx.mode != CAR_MODE_TURNING) return 0;
    return MODE_Turning_IsDone();
}

uint8_t Mode_IsBrakeDone(void)
{
    if (s_ctx.mode != CAR_MODE_BRAKE) return 0;
    return MODE_Brake_IsDone();
}
