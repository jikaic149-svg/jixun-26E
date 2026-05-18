#include "app_motor.h"
#include "app_config.h"
#include "algorithm/pid_controller.h"
#include "broker/broker.h"



/* ---- 速度范围枚举 ---- */
typedef enum { RANGE_LOW = 0, RANGE_HIGH = 1 } SpeedRange_e;

static PIDInstance   s_pid_l;
static PIDInstance   s_pid_r;
static SpeedRange_e  s_range_l = RANGE_LOW;
static SpeedRange_e  s_range_r = RANGE_LOW;

static float s_target_l      = 0.0f;
static float s_target_r      = 0.0f;
static float s_smooth_l      = 0.0f;
static float s_smooth_r      = 0.0f;
static float s_prev_target_l = 0.0f;
static float s_prev_target_r = 0.0f;
static float s_cur_pwm_l     = 0.0f;
static float s_cur_pwm_r     = 0.0f;

/* 根据绝对目标速度填充 PID 初始化配置 */
static void fill_pid_cfg(PID_Init_Config_s *cfg, float abs_target)
{
    if (abs_target > SPEED_THR) {
        cfg->Kp            = PID_WHEEL_HI_KP;
        cfg->Ki            = PID_WHEEL_HI_KI;
        cfg->Kd            = PID_WHEEL_HI_KD;
        cfg->MaxOut        = PID_WHEEL_HI_MAX_OUT;
        cfg->IntegralLimit = PID_WHEEL_HI_INT_LIM;
        cfg->DeadBand      = PID_WHEEL_HI_DEADBAND;
    } else {
        cfg->Kp            = PID_WHEEL_LO_KP;
        cfg->Ki            = PID_WHEEL_LO_KI;
        cfg->Kd            = PID_WHEEL_LO_KD;
        cfg->MaxOut        = PID_WHEEL_LO_MAX_OUT;
        cfg->IntegralLimit = PID_WHEEL_LO_INT_LIM;
        cfg->DeadBand      = PID_WHEEL_LO_DEADBAND;
    }
    cfg->Improve = PID_Integral_Limit;
}

/* 检测目标速度是否跨越量程阈值，若切换则重新初始化 PID（清除积分） */
static void maybe_switch_range(PIDInstance *pid, float target, SpeedRange_e *range)
{
    float abs_t = (target < 0.0f) ? -target : target;
    SpeedRange_e new_range = (abs_t > SPEED_THR) ? RANGE_HIGH : RANGE_LOW;
    if (new_range != *range) {
        PID_Init_Config_s cfg;
        fill_pid_cfg(&cfg, abs_t);
        PIDInit(pid, &cfg);
        *range = new_range;
    }
}

/* ---- 辨识序列（修改为 1000mm/s 上限） ---- */
typedef struct {
    float vL;
    float vR;
    uint32_t duration_ms;
} IdentStep_t;

static const IdentStep_t ident_seq[] = {
    { 200, 0, 2000}, {   0, 0, 1500},
    { 500, 0, 2000}, {   0, 0, 1500},
    { 800, 0, 2000}, {   0, 0, 1500},
    {1000, 0, 2000}, {   0, 0, 1500},

    { 0,  200, 2000}, { 0, 0, 1500},
    { 0,  500, 2000}, { 0, 0, 1500},
    { 0,  800, 2000}, { 0, 0, 1500},
    { 0, 1000, 2000}, { 0, 0, 1500},

    {-200, 0, 2000}, {   0, 0, 1500},
    {-500, 0, 2000}, {   0, 0, 1500},
    {-800, 0, 2000}, {   0, 0, 1500},
    {-1000,0, 2000}, {   0, 0, 1500},

    { 0, -200, 2000}, { 0, 0, 1500},
    { 0, -500, 2000}, { 0, 0, 1500},
    { 0, -800, 2000}, { 0, 0, 1500},
    { 0,-1000, 2000}, { 0, 0, 1500},
};
#define IDENT_SEQ_LEN (sizeof(ident_seq) / sizeof(ident_seq[0]))

static bool     ident_running  = false;
static uint32_t ident_step     = 0;
static float    ident_timer_s  = 0.0f;

/* ================================================================
 * 公共接口实现
 * ================================================================ */

void App_Motor_Init(void)
{
    PID_Init_Config_s cfg;
    fill_pid_cfg(&cfg, 0.0f);   /* 初始使用低速参数 */
    PIDInit(&s_pid_l, &cfg);
    PIDInit(&s_pid_r, &cfg);
    s_range_l = RANGE_LOW;
    s_range_r = RANGE_LOW;
    s_target_l = 0.0f;
    s_target_r = 0.0f;
    s_smooth_l = 0.0f;
    s_smooth_r = 0.0f;
}

void App_Motor_StartIdentSequence(void)
{
    ident_running = true;
    ident_step    = 0;
    ident_timer_s = 0.0f;
    App_Motor_Init();
    App_Motor_Stop();
}

void App_Motor_Update(float dt_s)
{
    if (dt_s <= 0.0f) return;

    /* 从消息总线读取目标速度 */
    if (!ident_running) {
        s_target_l = Broker_Subscribe_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT);
        s_target_r = Broker_Subscribe_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT);
    }

    /* 辨识序列推进 */
    if (ident_running) {
        ident_timer_s += dt_s;
        float step_dur_s = (float)ident_seq[ident_step].duration_ms / 1000.0f;
        if (ident_timer_s >= step_dur_s) {
            ident_timer_s = 0.0f;
            ident_step++;
            if (ident_step >= IDENT_SEQ_LEN) {
                ident_running = false;
                App_Motor_Stop();
            }
        }
        if (ident_running) {
            s_target_l = ident_seq[ident_step].vL;
            s_target_r = ident_seq[ident_step].vR;
            // 同步发布到总线，以便 UI 或其他模块也能看到辨识时的目标速度
            Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT, s_target_l);
            Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, s_target_r);
        }
    }

    float spd_l = Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
    float spd_r = Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);

    /* 目标降至 0 时清除积分并立即重置平滑量，防止 FF 持续驱动 */
    if (s_target_l == 0.0f && s_prev_target_l != 0.0f) {
        PID_Init_Config_s cfg; fill_pid_cfg(&cfg, 0.0f);
        PIDInit(&s_pid_l, &cfg); s_range_l = RANGE_LOW;
        s_smooth_l = 0.0f;
    }
    if (s_target_r == 0.0f && s_prev_target_r != 0.0f) {
        PID_Init_Config_s cfg; fill_pid_cfg(&cfg, 0.0f);
        PIDInit(&s_pid_r, &cfg); s_range_r = RANGE_LOW;
        s_smooth_r = 0.0f;
    }
    s_prev_target_l = s_target_l;
    s_prev_target_r = s_target_r;

    /* 指数平滑：每周期追赶 CURVE_ALPHA 比例，停车时已在上方清零 */
    if (s_target_l != 0.0f)
        s_smooth_l += (s_target_l - s_smooth_l) * CURVE_ALPHA;
    if (s_target_r != 0.0f)
        s_smooth_r += (s_target_r - s_smooth_r) * CURVE_ALPHA;

    /* 目标速度跨阈值时切换参数集（以平滑后的目标判断量程） */
    maybe_switch_range(&s_pid_l, s_smooth_l, &s_range_l);
    maybe_switch_range(&s_pid_r, s_smooth_r, &s_range_r);

    /* 前馈 + PID 修正（均跟随平滑目标） */
    float pid_l = PIDCalculate(&s_pid_l, spd_l, s_smooth_l, dt_s);
    float pid_r = PIDCalculate(&s_pid_r, spd_r, s_smooth_r, dt_s);
    float pwm_l = s_smooth_l * PID_WHEEL_FF + pid_l;
    float pwm_r = s_smooth_r * PID_WHEEL_FF + pid_r;

    /* 停车死区：目标为 0 且车速极低时强制切断，防低速振荡 */
    if (s_target_l == 0.0f &&
        spd_l > -WHEEL_STOP_THR && spd_l < WHEEL_STOP_THR) {
        pwm_l = 0.0f;
    }
    if (s_target_r == 0.0f &&
        spd_r > -WHEEL_STOP_THR && spd_r < WHEEL_STOP_THR) {
        pwm_r = 0.0f;
    }

    Broker_Publish_Float(TOPIC_PWM_LEFT, pwm_l);
    Broker_Publish_Float(TOPIC_PWM_RIGHT, pwm_r);
}

float App_Motor_GetSpeedL(void)
{
    return Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
}

float App_Motor_GetSpeedR(void)
{
    return Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);
}

void App_Motor_SetSpeed(float left_pwm, float right_pwm)
{
    Broker_Publish_Float(TOPIC_PWM_LEFT, left_pwm);
    Broker_Publish_Float(TOPIC_PWM_RIGHT, right_pwm);
}

float App_Motor_GetPwmL(void) { return Broker_Subscribe_Float(TOPIC_PWM_LEFT); }
float App_Motor_GetPwmR(void) { return Broker_Subscribe_Float(TOPIC_PWM_RIGHT); }

void App_Motor_SetTargetSpeed(float left_mm_s, float right_mm_s)
{
    /* 速度上限 1000 mm/s，下限不低于 -1000 mm/s */
    if (left_mm_s  >  1000.0f) left_mm_s  =  1000.0f;
    if (left_mm_s  < -1000.0f) left_mm_s  = -1000.0f;
    if (right_mm_s >  1000.0f) right_mm_s =  1000.0f;
    if (right_mm_s < -1000.0f) right_mm_s = -1000.0f;
    
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT, left_mm_s);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, right_mm_s);
}

void App_Motor_Stop(void)
{
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_LEFT, 0.0f);
    Broker_Publish_Float(TOPIC_TARGET_WHEEL_SPEED_RIGHT, 0.0f);
    s_target_l = 0.0f;
    s_target_r = 0.0f;
    /* 重新初始化 PID 清除积分，防止下次起步暴冲 */
    PID_Init_Config_s cfg;
    fill_pid_cfg(&cfg, 0.0f);
    PIDInit(&s_pid_l, &cfg);
    PIDInit(&s_pid_r, &cfg);
    s_range_l = RANGE_LOW;
    s_range_r = RANGE_LOW;
    s_smooth_l = 0.0f;
    s_smooth_r = 0.0f;
    App_Motor_SetSpeed(0.0f, 0.0f);
}
