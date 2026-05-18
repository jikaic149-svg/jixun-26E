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

// 速度与几何补偿核心参数（随时可调）
#define TT_STRAIGHT_SPEED   300.0f  // 冲刺极速：大幅提高至 300mm/s
#define TT_CURVE_SPEED      220.0f  // 过弯速度：提升至 220mm/s
#define TT_ACCEL_STEP       8.0f    // 出弯加速步长：提速更快
#define TT_DECEL_STEP       10.0f   // 入弯减速步长：刹车更干脆
#define TT_EARLY_TURN_DIST  116.0f  // 120切内圈，112抛外圈，取中间值 116.0f 定海神针
#define TT_O_TO_A_DIST      250.0f  // O到A的实际起步距离 (210mm 基础 + 40mm 防压线后延补偿)

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
    
    float turn_angle = 60.0f;

    if (has_left && !has_right) {
        turn_angle = 60.0f;
        s_pending_led_color = -1;
    } else if (!has_left && has_right) {
        turn_angle = -60.0f;
        s_pending_led_color = -1;
    } else {
        if (vis_color == 1) { // 红色右转
            turn_angle = -60.0f;
            s_pending_led_color = 1; // 仅记录，坚决不亮灯！
        } else if (vis_color == 0) { // 绿色左转
            turn_angle = 60.0f;
            s_pending_led_color = 0;
        } else {
            int best_heading = left_heading;
            int min_dist = 999;
            for (int i = 0; i < 3; i++) {
                if (hex_graph[current_y_node].heading[i] == left_heading) {
                    int n = hex_graph[current_y_node].neighbor[i];
                    if (dist_to_O[n] < min_dist) { min_dist = dist_to_O[n]; best_heading = left_heading; }
                }
                if (hex_graph[current_y_node].heading[i] == right_heading) {
                    int n = hex_graph[current_y_node].neighbor[i];
                    if (dist_to_O[n] < min_dist) { min_dist = dist_to_O[n]; best_heading = right_heading; }
                }
            }
            turn_angle = (best_heading == left_heading) ? 60.0f : -60.0f;
            s_pending_led_color = -1;
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

    if (!s_inspect_end_sent && s_targets_recognized >= 4) {
        int next_node = -1;
        for (int i = 0; i < 3; i++) {
            if (hex_graph[s_graph_curr_node].heading[i] == s_graph_heading) {
                next_node = hex_graph[s_graph_curr_node].neighbor[i];
                break;
            }
        }
        if (next_node == NODE_O || s_graph_curr_node == NODE_O) {
            BSP_UART1_SendString("INSPECT_END\n");
            s_inspect_end_sent = 1;
        }
    }

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
        if (!s_turn_signal_sent && fabsf(wz) > 80.0f) {
            BSP_UART1_SendString("TURN\n");
            s_turn_signal_sent = 1;
            
            // 只有在车头真正甩起来的这一瞬间，才点亮对应路标的灯光并鸣笛两秒！
            if (s_pending_led_color == 1) { // 红色
                s_active_ui_color = 1;
                Device_WS2812_SetAll(255, 0, 0);
                Device_WS2812_Show();
                Device_Buzzer_Set(1); // 响蜂鸣器
                s_buzzer_timer = 2000; // 红色长鸣/亮灯 2000ms
            } else if (s_pending_led_color == 0) { // 绿色
                s_active_ui_color = 0;
                Device_WS2812_SetAll(0, 255, 0);
                Device_WS2812_Show();
                Device_Buzzer_Set(1); // 响蜂鸣器
                s_buzzer_timer = 2000; // 绿色断续响/亮灯 2000ms
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
            // 极限平滑软启动防甩尾
            if (s_current_speed < TT_STRAIGHT_SPEED) s_current_speed += TT_ACCEL_STEP;
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            // 几何切弯：提前切入外圈 (带防压线补偿)
            if (fabsf(odo - s_start_odo) >= (TT_O_TO_A_DIST - TT_EARLY_TURN_DIST)) {
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
            if (s_current_speed > TT_CURVE_SPEED) {
                s_current_speed -= TT_DECEL_STEP;
                if (s_current_speed < TT_CURVE_SPEED) s_current_speed = TT_CURVE_SPEED;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;
            
            if (fabsf(yaw_err) <= 5.0f) { 
                // 几何补偿
                s_start_odo = odo - TT_EARLY_TURN_DIST; 
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
            // 出弯后软启动重新加速到极速
            if (s_current_speed < TT_STRAIGHT_SPEED) s_current_speed += TT_ACCEL_STEP;
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            if (fabsf(odo - s_start_odo) >= (400.0f - TT_EARLY_TURN_DIST)) {
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
            if (s_current_speed > TT_CURVE_SPEED) {
                s_current_speed -= TT_DECEL_STEP;
                if (s_current_speed < TT_CURVE_SPEED) s_current_speed = TT_CURVE_SPEED;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;
            
            if (fabsf(yaw_err) <= 5.0f) {
                s_start_odo = odo - TT_EARLY_TURN_DIST;
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
            if (s_current_speed > TT_CURVE_SPEED) {
                s_current_speed -= TT_DECEL_STEP;
                if (s_current_speed < TT_CURVE_SPEED) s_current_speed = TT_CURVE_SPEED;
            }
            Mode_SetCurve(s_current_speed, s_target_yaw);

            yaw_err = s_target_yaw - current_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;

            if (fabsf(yaw_err) <= 5.0f) {
                s_start_odo = odo - TT_EARLY_TURN_DIST;
                Mode_SetStraight(s_current_speed, s_target_yaw);
                s_state = TT_STATE_A_TO_O_STRAIGHT;
                
                // 强制清空视觉残影 (不再强制熄灯，交由两秒定时器处理)
                s_pending_led_color = -1;
                
                s_turn_signal_sent = 0;
                Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
                BSP_UART1_SendString("TURN_END\n");
            }
            break;
            
        case TT_STATE_A_TO_O_STRAIGHT:
            if (s_current_speed < TT_STRAIGHT_SPEED) s_current_speed += TT_ACCEL_STEP;
            Mode_SetStraight(s_current_speed, s_target_yaw);
            
            // 终点刹车：带防压线补偿
            if (fabsf(odo - s_start_odo) >= TT_O_TO_A_DIST) {
                Mode_SetBrake();
                s_state = TT_STATE_BRAKE_FINAL;
                s_tick = 0;
            }
            break;
            
        case TT_STATE_BRAKE_FINAL:
            if (Mode_IsBrakeDone() || s_tick > 200) {
                Mode_SetDone();
                s_state = TT_STATE_DONE;
            }
            break;
            
        case TT_STATE_DONE:
            break;
    }
}
