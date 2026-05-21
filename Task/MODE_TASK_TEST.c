#include "MODE_TASK_TEST.h"
#include "../Mode/mode_manager.h"
#include "../APP/broker/broker.h"

#include <math.h>
#include <stdio.h>

typedef enum {
    TT_STATE_IDLE = 0,
    TT_STATE_O_TO_A,
    TT_STATE_TURN_AT_A_IN,
    TT_STATE_HEX_STRAIGHT,
    TT_STATE_HEX_TURN,
    TT_STATE_A_TO_O_TURN,
    TT_STATE_A_TO_O_STRAIGHT,
    TT_STATE_BRAKE_FINAL,
    TT_STATE_DONE
} TT_State_e;

// =========================================================
// 【TaskTest 全局核心参数配置区 (Config)】
// =========================================================
// 1. 速度与运动控制
#define CFG_SPEED_STRAIGHT      300.0f  // 直线冲刺极速 (mm/s)
#define CFG_SPEED_CURVE         220.0f  // 弯道平稳速度 (mm/s)
#define CFG_ACCEL_STEP          8.0f    // 加速步长 (越大加速越猛)
#define CFG_DECEL_STEP          10.0f   // 减速步长 (越大刹车越急)

// 2. 几何距离与航向
#define CFG_DIST_O_TO_A         250.0f  // 起步 O到A 的目标距离 (210mm基础 + 40mm防压线补偿)
#define CFG_DIST_HEX_SIDE       400.0f  // 六边形标准单边长 (mm)
#define CFG_DIST_EARLY_TURN     116.0f  // 提前入弯补偿距离 (允许单片机稍晚读取视觉，免受遮挡)
#define CFG_ANGLE_TURN          60.0f   // 六边形标准转向角度 (度)
#define CFG_YAW_ERROR_TOLERANCE 2.0f    // 出弯航向锁定容忍度 (度，极其严苛地对正，防止整体偏置拉伸)

// [Step 2.1 - 实跑回退] 架构 284.52 走多了，168 走少了，取中点 226 试探。
// 首段/末段只发生 1 次，累计误差小，暂保留架构几何值；hex 边重复 6 次，先压低。
// 调参原则：增大 → 入弯延后、直线更长；减小 → 入弯提前、直线更短。
#define CFG_DIST_TANGENT        57.74f                                                  /* L_T 几何切线长 (R=100, θ=60°) */
#define CFG_DIST_O_TO_A_PHYS    210.0f                                                  /* O ↔ A 物理距离 */
#define CFG_DIST_FIRST_STRAIGHT (CFG_DIST_O_TO_A_PHYS - CFG_DIST_TANGENT)               /* 152.26mm: O→A 首段 */
#define CFG_DIST_HEX_STRAIGHT   198.0f                                                  /* hex 边手调：二分逼近 (188~209) */
#define CFG_DIST_FINAL_EXTRA    200.0f                                                  /* 末段额外推进 (越线裕量) */
#define CFG_DIST_FINAL_STRAIGHT (CFG_DIST_O_TO_A_PHYS - CFG_DIST_TANGENT + CFG_DIST_FINAL_EXTRA) /* 352.26mm: A→O 末段 */

// 3. 时序与传感器判定阈值
#define CFG_TICK_START_DELAY    50      // 起步原地等待视觉识别的超时 tick (50 * 5ms = 250ms)
#define CFG_TICK_BRAKE_TIMEOUT  200     // 终点强制刹车超时 tick (200 * 5ms = 1000ms)
#define CFG_GYRO_WZ_THRESHOLD   80.0f   // 陀螺仪Z轴角速度触发阈值 (过滤直线颠簸，准确捕捉转弯甩尾)
#define CFG_UI_LED_BUZZER_TIME  2000    // 路标识别成功时，LED与蜂鸣器持续发声的毫秒数
// =========================================================

static TT_State_e s_state;
static uint32_t   s_tick = 0;
static float      s_start_odo = 0.0f;
static float      s_start_yaw = 0.0f;
static float      s_target_yaw = 0.0f;
static float      s_current_speed = 0.0f;
static uint8_t    s_hex_edge_idx = 0;
static uint32_t   s_buzzer_timer = 0;
static int        s_active_ui_color = -1; // 记录当前UI显示的颜色(为了实现断续响)

#include "../Device/device_ws2812.h"
#include "../Device/device_buzzer.h"
#include "../BSP/bsp_uart.h"

#define NODE_O 0
#define NODE_A 1
#define NODE_G 2
#define NODE_F 3
#define NODE_E 4
#define NODE_T1 5
#define NODE_T2 6
#define NODE_M 7
#define NODE_B2 8
#define NODE_B1 9
#define NODE_D 10
#define NODE_C 11
#define NODE_B 12
#define NODE_H 13

typedef struct {
  int neighbor[3];
  int heading[3];
} HexNode_s;

static const HexNode_s hex_graph[14] = {
    {{NODE_A, -1, -1}, {3, -1, -1}},
    {{NODE_O, NODE_G, NODE_B}, {0, 2, 4}},
    {{NODE_A, NODE_F, -1}, {5, 3, -1}},
    {{NODE_G, NODE_E, NODE_H}, {0, 2, 4}},
    {{NODE_F, NODE_T1, -1}, {5, 3, -1}},
    {{NODE_E, NODE_T2, -1}, {0, 4, -1}},
    {{NODE_T1, NODE_M, -1}, {1, 5, -1}},
    {{NODE_T2, NODE_H, NODE_B2}, {2, 0, 4}},
    {{NODE_M, NODE_B1, -1}, {1, 5, -1}},
    {{NODE_B2, NODE_D, -1}, {2, 0, -1}},
    {{NODE_B1, NODE_C, -1}, {3, 1, -1}},
    {{NODE_D, NODE_H, NODE_B}, {4, 2, 0}},
    {{NODE_C, NODE_A, -1}, {3, 1, -1}},
    {{NODE_M, NODE_F, NODE_C}, {3, 1, 5}}};

static const int dist_to_O[14] = {
    0, 1, 2, 3, 4, 5, 6, 5, 6, 5, 4, 3, 2, 4
};

static int s_graph_curr_node = NODE_O;
static int s_graph_heading = 3;
static int s_targets_recognized = 0;
static uint8_t s_inspect_end_sent = 0;
static uint8_t s_turn_signal_sent = 0;
static int s_pending_led_color = -1; // -1:无, 0:绿, 1:红
static int s_is_y_junction = 0;
static int s_last_y_node = -1;
static int s_last_y_choice = -1;
static uint8_t s_car_end_sent = 0;

static uint8_t s_node_visit_count[14][6] = {{0}};
static int s_node_last_choice[14][6] = {{0}};
static int s_is_clockwise = -1; // -1: 未判定, 1: 顺时针(去G), 0: 逆时针(去B)

static float GetNextTurnAngle(void) {
    int current_y_node = -1;
    for (int i = 0; i < 3; i++) {
        if (hex_graph[s_graph_curr_node].heading[i] == s_graph_heading) {
            current_y_node = hex_graph[s_graph_curr_node].neighbor[i];
            break;
        }
    }
    
    if (current_y_node == -1) current_y_node = NODE_A;

    int left_heading = (s_graph_heading + 1) % 6;
    int right_heading = (s_graph_heading + 5) % 6;
    int has_left = 0, has_right = 0;
    
    for (int i = 0; i < 3; i++) {
        int h = hex_graph[current_y_node].heading[i];
        if (h == left_heading) has_left = 1;
        if (h == right_heading) has_right = 1;
    }

    // 重置并判断是否为 Y 字路口（必须同时有左和右的路才是 Y 字分岔）
    s_is_y_junction = (has_left && has_right) ? 1 : 0;
    
    int vis_color = -1;
    // 关键修正：仅在真正的 Y 字路口，才去消费视觉数据！
    if (s_is_y_junction) {
        vis_color = Broker_Subscribe_Int(TOPIC_VISION_COLOR);
        // 读完立刻重置为 -1，满足你的“必须清零”要求，防止跨 Y 弯污染！
        Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
    }
    
    float turn_angle = CFG_ANGLE_TURN;

    if (has_left && !has_right) {
        turn_angle = CFG_ANGLE_TURN;
        s_pending_led_color = -1;
    } else if (!has_left && has_right) {
        turn_angle = -CFG_ANGLE_TURN;
        s_pending_led_color = -1;
    } else {
        s_node_visit_count[current_y_node][s_graph_heading]++; // 记录在该节点该进入方向的访问次数
        
        if (vis_color == 1) { // 红色右转
            turn_angle = -CFG_ANGLE_TURN;
            s_pending_led_color = 1; // 仅记录，坚决不亮灯！
            s_node_last_choice[current_y_node][s_graph_heading] = right_heading; // 记录在该进入方向选过的方向
        } else if (vis_color == 0) { // 绿色左转
            turn_angle = CFG_ANGLE_TURN;
            s_pending_led_color = 0;
            s_node_last_choice[current_y_node][s_graph_heading] = left_heading; // 记录在该进入方向选过的方向
        } else {
            int left_dist = 999;
            int right_dist = 999;
            for (int i = 0; i < 3; i++) {
                if (hex_graph[current_y_node].heading[i] == left_heading) {
                    int n = hex_graph[current_y_node].neighbor[i];
                        left_dist = dist_to_O[n];
                }
                if (hex_graph[current_y_node].heading[i] == right_heading) {
                    int n = hex_graph[current_y_node].neighbor[i];
                        right_dist = dist_to_O[n];
                }
            }
            int best_heading = left_heading;
            
            // [全新探索算法] 
            // A点(回城大门)特殊处理：在A点绝对不能瞎逛探索！强制最高优先级使用最短路径回原点。
            if (current_y_node != NODE_A && s_node_visit_count[current_y_node][s_graph_heading] >= 2 && s_node_last_choice[current_y_node][s_graph_heading] != -1) {
                best_heading = (s_node_last_choice[current_y_node][s_graph_heading] == left_heading) ? right_heading : left_heading;
            } else {
                // 第一次来到这个节点，或者是A点，永远遵守最短路径原则保证回家效率
                if (left_dist < right_dist) {
                    best_heading = left_heading;
                } else if (right_dist < left_dist) {
                    best_heading = right_heading;
                } else {
                    if (s_is_clockwise == 0) {
                        best_heading = right_heading; // 逆时针时，距离相等默认选右
                    } else {
                        best_heading = left_heading;  // 顺时针或未判定时，距离相等默认选左
                    }
                }
            }
            
            s_node_last_choice[current_y_node][s_graph_heading] = best_heading; // 更新在该进入方向的最新选择
            turn_angle = (best_heading == left_heading) ? CFG_ANGLE_TURN : -CFG_ANGLE_TURN;
            s_pending_led_color = -1;
        }
        if (current_y_node == NODE_A && s_is_clockwise == -1) {
            if (turn_angle > 0) s_is_clockwise = 0; // 左拐去 NODE_B，为逆时针
            else s_is_clockwise = 1;                // 右拐去 NODE_G，为顺时针
        }
        if (s_targets_recognized < 4) s_targets_recognized++;
    }

    // 关键回传：单片机切弯瞬间的数据来源和结果！
    char log_buf[128];
    if (s_is_y_junction) {
        snprintf(log_buf, sizeof(log_buf), "TURN_DECISION: Type=Y, VisRead=%d, Src=%s, Angle=%.0f\n", 
            vis_color, 
            (vis_color != -1) ? "VISION" : "DIJKSTRA_FALLBACK",
            turn_angle);
    } else {
        snprintf(log_buf, sizeof(log_buf), "TURN_DECISION: Type=L, VisRead=-1, Src=MAP_ONLY, Angle=%.0f\n", turn_angle);
    }
    BSP_UART1_SendString(log_buf);

    s_graph_curr_node = current_y_node;
    if (turn_angle > 0) s_graph_heading = (s_graph_heading + 1) % 6;
    else s_graph_heading = (s_graph_heading + 5) % 6;

    // 删除了原有的达到4个目标即发送INSPECT_END的逻辑，改为在停车时发送end信号

    // 重置陀螺仪发送标志位，准备迎接下一次过弯的真实物理转向
    s_turn_signal_sent = 0;

    return turn_angle;
}

/* 辅助函数：获取当前总里程 (mm) */
static float GetOdo(void) {
    float dl = Broker_Subscribe_Float(TOPIC_DISTANCE_LEFT);
    float dr = Broker_Subscribe_Float(TOPIC_DISTANCE_RIGHT);
    return (dl + dr) / 2.0f;
}

void TaskTest_Init(void) {
    s_state = TT_STATE_IDLE;
    s_tick  = 0;
    s_hex_edge_idx = 0;
    s_current_speed = 0.0f;
    s_graph_curr_node = NODE_O;
    s_graph_heading = 3;
    s_targets_recognized = 0;
    s_inspect_end_sent = 0;
    s_car_end_sent = 0;
    s_last_y_node = -1;
    s_last_y_choice = -1;
    s_is_clockwise = -1;
    
    // 清空图遍历状态
    for (int i = 0; i < 14; i++) {
        for (int h = 0; h < 6; h++) {
            s_node_visit_count[i][h] = 0;
            s_node_last_choice[i][h] = -1;
        }
    }
    
    Device_WS2812_Clear();
    Device_WS2812_Show();
    s_buzzer_timer = 0;
    s_active_ui_color = -1;
    Device_Buzzer_Set(0);
}

void TaskTest_Exec(void) {
    s_tick++;
    float current_yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
    float odo = GetOdo();
    float yaw_err = 0.0f;
    // 无阻塞蜂鸣器与灯光倒计时 (APP_TICK_MS = 5ms)
    if (s_buzzer_timer > 0) {
        if (s_buzzer_timer > 5) {
            s_buzzer_timer -= 5;
            
            // 绿色断续响，红色长响
            if (s_active_ui_color == 0) { // 绿色
                // 每 200ms 一个周期：前 100ms 响，后 100ms 停
                if ((s_buzzer_timer % 200) >= 100) {
                    Device_Buzzer_Set(1);
                } else {
                    Device_Buzzer_Set(0);
                }
            } else if (s_active_ui_color == 1) { // 红色
                Device_Buzzer_Set(1); // 保持长响
            }
            
        } else {
            s_buzzer_timer = 0;
            s_active_ui_color = -1;
            Device_Buzzer_Set(0);  // 关蜂鸣器
            Device_WS2812_Clear(); // 关灯
            Device_WS2812_Show();
        }
    }
    
    // 如果处于正在过弯的状态，通过陀螺仪 Z 轴角速度精准捕捉物理转向的瞬间
    if (s_state == TT_STATE_TURN_AT_A_IN || 
        s_state == TT_STATE_HEX_TURN || 
        s_state == TT_STATE_A_TO_O_TURN) {
        
        float wz = Broker_Subscribe_Float(TOPIC_JY61P_WZ);
        // 强化判定：提高到 80.0f 彻底免疫直线跑动中的任何颠簸晃动！
        if (!s_turn_signal_sent && fabsf(wz) > CFG_GYRO_WZ_THRESHOLD) {
            BSP_UART1_SendString("TURN\n");
            s_turn_signal_sent = 1;
            
            // 只有在车头真正甩起来的这一瞬间，才点亮对应路标的灯光并鸣笛两秒！
            if (s_pending_led_color == 1) { // 红色
                s_active_ui_color = 1;
                Device_WS2812_SetAll(255, 0, 0);
                Device_WS2812_Show();
                Device_Buzzer_Set(1); // 响蜂鸣器
                s_buzzer_timer = CFG_UI_LED_BUZZER_TIME; // 红色长鸣/亮灯 2000ms
            } else if (s_pending_led_color == 0) { // 绿色
                s_active_ui_color = 0;
                Device_WS2812_SetAll(0, 255, 0);
                Device_WS2812_Show();
                Device_Buzzer_Set(1); // 响蜂鸣器
                s_buzzer_timer = CFG_UI_LED_BUZZER_TIME; // 绿色断续响/亮灯 2000ms
            } else {
                // 假如不到两秒又触发一次转向且没有目标，直接抢断关闭 UI 显示
                Device_WS2812_Clear();
                Device_WS2812_Show();
                Device_Buzzer_Set(0);
                s_buzzer_timer = 0;
            }
        }
    }

    switch (s_state) {
        case TT_STATE_IDLE:
            // 记录发车时绝对航向
            s_start_yaw  = current_yaw;
            s_target_yaw = current_yaw;
            s_start_odo  = odo;
            s_hex_edge_idx = 0;
            s_current_speed = 0.0f;
            
            // 1. 发车：从 O 点直行到 A 点
            s_state = TT_STATE_O_TO_A;
            BSP_UART1_SendString("INSPECT_START\n");
            break;
            
        case TT_STATE_O_TO_A:
            // 【极其关键】由于视觉在等待发车阶段处于休眠无识别状态，
            // 收到发车信号后需要几十毫秒“睁眼”并处理第一帧发送。
            // 但起步距离到判定线仅仅 15cm，一脚油门瞬间冲过导致视觉跟不上。
            // 因此起步原地强行憋 500ms (50 个 tick) 留给视觉充分的识别时间！
            if (s_tick < CFG_TICK_START_DELAY) {
                Mode_SetStraight(0, s_target_yaw);
                s_start_odo = GetOdo(); // 不断刷新起始里程，防止抖动
                break;
            }
            
            // 软起步加速
            if (s_current_speed < CFG_SPEED_STRAIGHT) s_current_speed += CFG_ACCEL_STEP;
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            // [双状态机硬隔离] 直线段以 clean anchor 测量物理推进距离
            if (fabsf(odo - s_start_odo) >= CFG_DIST_FIRST_STRAIGHT) {
                float turn_angle = GetNextTurnAngle();
                s_target_yaw = s_start_yaw + turn_angle; 
                if (s_target_yaw > 180.0f) s_target_yaw -= 360.0f;
                if (s_target_yaw < -180.0f) s_target_yaw += 360.0f;
                
                // 不再直接突变到低速，而是带着当前的高速切入弯道状态，由弯道状态接管平滑减速
                Mode_SetCurve(s_current_speed, s_target_yaw);
                s_state = TT_STATE_TURN_AT_A_IN;
                
                if (s_is_y_junction) {
                    BSP_UART1_SendString("TURN_START\n");
                } else {
                    BSP_UART1_SendString("TURN_BLIND_ONLY\n");
                }
            }
            break;
            
        case TT_STATE_TURN_AT_A_IN:
            // 平滑减速至过弯速度
            if (s_current_speed > CFG_SPEED_CURVE) {
                s_current_speed -= CFG_DECEL_STEP;
                if (s_current_speed < CFG_SPEED_CURVE) s_current_speed = CFG_SPEED_CURVE;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;
            
            if (fabsf(yaw_err) <= CFG_YAW_ERROR_TOLERANCE) {
                // [双状态机硬隔离] 进入 STRAIGHT 瞬间 clean anchor，
                // 不再把弧线长度作为虚拟负初始里程塞进下一段
                s_start_odo = odo;
                s_hex_edge_idx = 0;
                Mode_SetStraight(s_current_speed, s_target_yaw);
                s_state = TT_STATE_HEX_STRAIGHT;
                
                s_turn_signal_sent = 0;
                // 转弯结束，强制清空视觉残影！(不再强制熄灯，交由两秒定时器处理)
                Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
                BSP_UART1_SendString("TURN_END\n");
            }
            break;
            
        case TT_STATE_HEX_STRAIGHT:
            // [加快角度回直] 出弯航向未对正前先不加速，保持 220mm/s 弯道速度让 PID 用更多 tick 收敛 yaw
            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f)  yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;
            if (fabsf(yaw_err) <= CFG_YAW_ERROR_TOLERANCE) {
                // 航向已对齐 → 可以放胆冲到极速
                if (s_current_speed < CFG_SPEED_STRAIGHT) s_current_speed += CFG_ACCEL_STEP;
            }
            // 否则 s_current_speed 保持出弯时的 CFG_SPEED_CURVE 220mm/s 不变
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            // [自适应物理补偿] 应对轮胎打滑与电池衰减带来的物理抛离误差
            // [微量物理补偿] 每弯温柔衰减 0.5f，完美压制轻微的打滑外抛
            // 设置了缩短上限（最多减 4.0mm），防止如果在里面多绕了几圈导致无限内切
            float decay_dist = s_hex_edge_idx * 0.5f;
            if (decay_dist > 4.0f) decay_dist = 4.0f; 
            float dynamic_target_dist = CFG_DIST_HEX_STRAIGHT - decay_dist;
            
            if (fabsf(odo - s_start_odo) >= dynamic_target_dist) {
                float turn_angle = GetNextTurnAngle();
                
                s_target_yaw += turn_angle;
                if (s_target_yaw > 180.0f) s_target_yaw -= 360.0f;
                if (s_target_yaw < -180.0f) s_target_yaw += 360.0f;
                
                Mode_SetCurve(s_current_speed, s_target_yaw);
                
                // 判断下一次是否走向 O 点
                int current_y_node = -1;
                for (int i = 0; i < 3; i++) {
                    if (hex_graph[s_graph_curr_node].heading[i] == s_graph_heading) {
                        current_y_node = hex_graph[s_graph_curr_node].neighbor[i];
                        break;
                    }
                }
                
                if (current_y_node == NODE_O) {
                    s_state = TT_STATE_A_TO_O_TURN;
                } else {
                    s_state = TT_STATE_HEX_TURN;
                }
                
                if (s_is_y_junction) {
                    BSP_UART1_SendString("TURN_START\n");
                } else {
                    BSP_UART1_SendString("TURN_BLIND_ONLY\n");
                }
            }
            break;
            
        case TT_STATE_HEX_TURN:
            // 平滑减速
            if (s_current_speed > CFG_SPEED_CURVE) {
                s_current_speed -= CFG_DECEL_STEP;
                if (s_current_speed < CFG_SPEED_CURVE) s_current_speed = CFG_SPEED_CURVE;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;
            
            if (fabsf(yaw_err) <= CFG_YAW_ERROR_TOLERANCE) {
                // [双状态机硬隔离] clean anchor 切回 STRAIGHT
                s_start_odo = odo;
                s_hex_edge_idx++; // 记录转弯次数，用于后续的里程递减补偿
                Mode_SetStraight(s_current_speed, s_target_yaw);
                s_state = TT_STATE_HEX_STRAIGHT;

                // 强制清空视觉残影，准备迎接下一个路口 (不再强制熄灯，交由两秒定时器处理)
                s_pending_led_color = -1;
                
                s_turn_signal_sent = 0;
                Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
                BSP_UART1_SendString("TURN_END\n");
            }
            break;

        case TT_STATE_A_TO_O_TURN:
            // 平滑减速
            if (s_current_speed > CFG_SPEED_CURVE) {
                s_current_speed -= CFG_DECEL_STEP;
                if (s_current_speed < CFG_SPEED_CURVE) s_current_speed = CFG_SPEED_CURVE;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;

            if (fabsf(yaw_err) <= CFG_YAW_ERROR_TOLERANCE) {
                // [双状态机硬隔离] clean anchor 切入末段直线
                s_start_odo = odo;
                Mode_SetStraight(s_current_speed, s_target_yaw);
                s_state = TT_STATE_A_TO_O_STRAIGHT;
                
                // 强制清空视觉残影 (不再强制熄灯，交由两秒定时器处理)
                s_pending_led_color = -1;
                
                s_turn_signal_sent = 0;
                Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
                BSP_UART1_SendString("TURN_END\n");
                
                // 极限保险机制：在 A 节点切弯结束、对准 O 点刚起步的瞬间，立刻发送 end 存盘！
                // 此时所有的色块都已遍历完毕，立刻交卷能防止归途撞墙或断电导致没存上！
                if (!s_car_end_sent) {
                    BSP_UART1_SendString("end\n");
                    s_car_end_sent = 1;
                }
            }
            break;
            
        case TT_STATE_A_TO_O_STRAIGHT:
            if (s_current_speed < CFG_SPEED_STRAIGHT) s_current_speed += CFG_ACCEL_STEP;
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            // 终点刹车：clean anchor 下的物理推进距离判定
            if (fabsf(odo - s_start_odo) >= CFG_DIST_FINAL_STRAIGHT) {
                Mode_SetBrake();
                s_state = TT_STATE_BRAKE_FINAL;
                s_tick = 0;
            }
            break;
            
        case TT_STATE_BRAKE_FINAL:
            if (Mode_IsBrakeDone() || s_tick > CFG_TICK_BRAKE_TIMEOUT) {
                Mode_SetDone();
                s_state = TT_STATE_DONE;
            }
            break;
            
        case TT_STATE_DONE:
            if (!s_car_end_sent) {
                BSP_UART1_SendString("end\n");
                s_car_end_sent = 1;
            }
            break;
    }
}
