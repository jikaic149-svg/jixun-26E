#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 系统级周期参数 */
#define APP_TICK_MS             5U
#define APP_TICK_S              (APP_TICK_MS / 1000.0f)

/* 通信参数 */
#define COMM_REPORT_INTERVAL_MS 50U     /* 轮速上报间隔 (50ms = 20Hz) */

/* 物理底盘机械参数 */
/* 编码器单圈综合脉冲 = 物理线数 500 * 减速比 20.409 * 4倍频 = 40818.0 */
#define CAR_ENCODER_RESOLUTION  (500.0f * 20.409f * 4.0f)
/* 轮胎直径 48mm * PI = 150.796mm */
#define CAR_WHEEL_CIRCUMFERENCE_MM 150.796f
/* 底盘电机 PWM 驱动占空比上限 */
#define CAR_MOTOR_MAX_DUTY      100

/* === 导航与模式控制参数 === */
/* 偏航角纠偏 PID (用于直线、曲线走直) */
#define PID_YAW_KP                  1.5f
#define PID_YAW_KI                  0.0f
#define PID_YAW_KD                  0.2f

/* 直线模式纠偏限幅 */
#define MODE_STRAIGHT_MAX_CORRECTION 200.0f
#define STRAIGHT_TURN_RATIO          0.5f

/* 弧线模式串级 PID 参数 (外环:角度->角速度, 内环:角速度->差速) */
#define PID_YAW_OUTER_KP             100.0f  /* 恢复极高增益以配合动态目标点，确保迅速输出目标角速度 */
#define PID_YAW_OUTER_KI             0.0f
#define PID_YAW_OUTER_MAX_WZ         160.0f  // 提高上限，配合 300mm/s 高速切弯，防止外抛
#define PID_WZ_INNER_KP              1.5f
#define PID_WZ_INNER_KI              0.1f
#define PID_WZ_INNER_MAX_DIFF        250.0f  // 提高差速上限，允许更剧烈的单侧轮减速/加速转向
#define PID_WZ_INNER_INT_LIMIT       100.0f

#define MODE_CURVE_MAX_CORRECTION    200.0f
#define CURVE_TURN_RATIO             0.5f

/* 原地转向模式参数 */
#define PID_TURNING_KD               2.0f
#define TURNING_MAX_SPEED            150.0f
#define TURNING_MIN_SPEED            50.0f
#define TURNING_DECEL_ZONE           45.0f

/* 刹车停稳参数 */
#define BRAKE_SPEED_THRESHOLD        10.0f
#define BRAKE_STABLE_COUNT           5

/* 其它功能参数可在此追加 */

/* ================================================================
 * 底盘轮速控制 PID 参数 (基于 25E 参数按量程比 100/999 缩放)
 * ================================================================ */
/* 低速段 (|target| ≤ SPEED_THR, 约 200~500 mm/s) */
#define PID_WHEEL_LO_KP         0.25f
#define PID_WHEEL_LO_KI         0.80f
#define PID_WHEEL_LO_KD         0.001f
#define PID_WHEEL_LO_MAX_OUT    100.0f
#define PID_WHEEL_LO_INT_LIM    100.0f
#define PID_WHEEL_LO_DEADBAND   2.0f

/* 高速段 (|target| > SPEED_THR, 约 500~1000 mm/s) */
#define PID_WHEEL_HI_KP         0.25f
#define PID_WHEEL_HI_KI         0.60f
#define PID_WHEEL_HI_KD         0.001f
#define PID_WHEEL_HI_MAX_OUT    100.0f
#define PID_WHEEL_HI_INT_LIM    100.0f
#define PID_WHEEL_HI_DEADBAND   2.0f

/* 速度量程切换阈值 (mm/s) */
#define SPEED_THR               500.0f

/* 前馈增益 (FF)：折中考虑空载与负载，设定为 0.18 */
#define PID_WHEEL_FF            0.18f

/* 停车死区：目标为 0 且实测低于此值时强制切断输出 */
#define WHEEL_STOP_THR          20.0f

/* 指数目标平滑系数：每 5ms 周期追赶 15% */
#define CURVE_ALPHA             0.15f

#endif // APP_CONFIG_H
