/**
 * @file  MODE_TASK_HEX.c
 * @brief 正六边形地图专属循迹与过弯任务
 */
#include "MODE_TASK_HEX.h"
#include "Application/broker/broker.h"
#include "BSP/bsp_timer.h"
#include "Device/jy61p/jy61p.h"
#include "Device/opencv/opencv.h"
#include "Device/track/track.h"
#include "Mode/MODE_STRAIGHT.h"
#include "Mode/MODE_TRACKING.h"
#include "Mode/MODE_TURNING.h"
#include "config.h"
#include "ti_msp_dl_config.h"


/* 外部引入底层红外实例以读取原始 ADC (不修改底层代码) */
extern Track_Instance g_track;

/* ======================== 阈值与参数配置 ======================== */
/* 外部探头通道 */
#define CH_LEFT_OUTER 0
#define CH_RIGHT_OUTER 7

/* 白底参考值 (来自最新 log.txt 实测纯白底均值) */
#define WHITE_BASE_L 2210
#define WHITE_BASE_R 2930

/* 触发阈值已废弃，现在由底层的 Track_GetNormalized(0.0~1.0) 统一处理抗干扰 */

/* 盲走屏蔽距离: 发车区(全黑)或刚转弯出路口时盲走屏蔽 120mm
 * (缩短死区，尽早吸附中心) */
#define HEX_EDGE_BLIND_DIST 120.0f

/* 单边触发后最大补偿等待距离 (mm) (恢复 90mm，确保单弯补偿足够) */
#define MAX_COMPENSATE_DIST 90.0f

/* 双边触发后，往前走多少 mm 让车轴中心对准 Y 型中心 (恢复到
 * 100mm，防止转弯太早导致出弯偏离中心) */
#define CENTER_OFFSET_DIST 100.0f

/* 旋转后直行脱离路口的盲走距离 (mm) (恢复到150，确保彻底跨过交叉线) */
#define EXIT_BLIND_DIST 150.0f

/* 最终回到 O 点：额外前行停车距离 (mm) */
#define HEX_FINAL_PARK_EXTRA_DIST 200.0f

/* 最终回到 O 点：停车前前行速度 (mm/s) */
#define HEX_FINAL_PARK_SPEED TASK_STRAIGHT_SLOW

/* ======================== 内部状态 ======================== */
/* 六边形拓扑地图定义 */
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
    /* 0: O  */ {{NODE_A, -1, -1}, {3, -1, -1}},
    /* 1: A  */ {{NODE_O, NODE_G, NODE_B}, {0, 2, 4}},
    /* 2: G  */ {{NODE_A, NODE_F, -1}, {5, 3, -1}},
    /* 3: F  */ {{NODE_G, NODE_E, NODE_H}, {0, 2, 4}},
    /* 4: E  */ {{NODE_F, NODE_T1, -1}, {5, 3, -1}},
    /* 5: T1 */ {{NODE_E, NODE_T2, -1}, {0, 4, -1}},
    /* 6: T2 */ {{NODE_T1, NODE_M, -1}, {1, 5, -1}},
    /* 7: M  */ {{NODE_T2, NODE_H, NODE_B2}, {2, 0, 4}},
    /* 8: B2 */ {{NODE_M, NODE_B1, -1}, {1, 5, -1}},
    /* 9: B1 */ {{NODE_B2, NODE_D, -1}, {2, 0, -1}},
    /* 10: D */ {{NODE_B1, NODE_C, -1}, {3, 1, -1}},
    /* 11: C */ {{NODE_D, NODE_H, NODE_B}, {4, 2, 0}},
    /* 12: B */ {{NODE_C, NODE_A, -1}, {3, 1, -1}},
    /* 13: H */ {{NODE_M, NODE_F, NODE_C}, {3, 1, 5}}};

static const int dist_to_O[14] = {
    0, // 0: O
    1, // 1: A
    2, // 2: G
    3, // 3: F
    4, // 4: E
    5, // 5: T1
    6, // 6: T2
    5, // 7: M
    6, // 8: B2
    5, // 9: B1
    4, // 10: D
    3, // 11: C
    2, // 12: B
    4  // 13: H
};

static int s_graph_curr_node = NODE_O;
static int s_graph_heading = 3;      /* 面朝西 (航向 3) */
static int s_targets_recognized = 0; /* 成功识别的图形数量 */
static uint8_t s_vision_cleared = 0; /* 视觉缓存清理标志 */

static int s_cached_vis_turn = -1;
static int s_cached_color = -1;
static int s_cached_shape = -1;

static int s_recognized_colors[4] = {-1, -1, -1, -1};
static int s_recognized_shapes[4] = {-1, -1, -1, -1};

typedef enum {
  HEX_START_BLIND = -1, /* 起步脱离O点黑块 */
  HEX_TRACKING = 0, /* 正常循迹 */
  HEX_COMPENSATE_L, /* 左侧率先触发，冻结航向直行等右侧 */
  HEX_COMPENSATE_R, /* 右侧率先触发，冻结航向直行等左侧 */
  HEX_CENTERING,    /* 均已触发，直行至旋转中心 */
  HEX_TURNING,      /* 陀螺仪闭环原地转弯 60°/-60° */
  HEX_EXITING,      /* 转完后直行脱离路口，防线干扰 */
  HEX_FINAL_PARKING /* 最终回到 O 点：再走一段再停车 */
} HexStep_e;

static HexStep_e s_step = HEX_TRACKING;
static uint8_t s_done = 0;
static float s_target_yaw = 0.0f; /* 锁定/目标航向 */
static float s_dist_snap = 0.0f;
static float s_current_blind_dist = 100.0f;
static uint8_t s_node_id = 1; /* 当前走过的路口节点 ID，用于发送给视觉模块 */
static uint8_t s_inspect_end_sent = 0;

/* ======================== 工具函数 ======================== */
static float _normalize(float a) {
  while (a > 180.0f)
    a -= 360.0f;
  while (a < -180.0f)
    a += 360.0f;
  return a;
}

static float _get_mileage(void) {
  float dl = Broker_Subscribe_Float(TOPIC_DISTANCE_LEFT);
  float dr = Broker_Subscribe_Float(TOPIC_DISTANCE_RIGHT);
  return (dl + dr) * 0.5f;
}

static void _snap_dist(void) { s_dist_snap = _get_mileage(); }

static float _get_dist(void) { return fabsf(_get_mileage() - s_dist_snap); }

static int _graph_get_neighbor(int node, int heading) {
  for (int i = 0; i < 3; i++) {
    if (hex_graph[node].heading[i] == heading) {
      return hex_graph[node].neighbor[i];
    }
  }
  return -1;
}

static void _bt_print_targets(void) {
  const char *c_str[4];
  const char *s_str[4];
  for (int i = 0; i < 4; i++) {
    c_str[i] = (s_recognized_colors[i] == 1)
                   ? "Red"
                   : ((s_recognized_colors[i] == 0) ? "Green" : "Unknown");
    s_str[i] = (s_recognized_shapes[i] == 1)
                   ? "Circle"
                   : ((s_recognized_shapes[i] == 0) ? "Square" : "Unknown");
  }

  static char print_buf[256];
  int len = snprintf(print_buf, sizeof(print_buf),
                     "Return to O. Targets:\r\n"
                     "1: %s %s\r\n"
                     "2: %s %s\r\n"
                     "3: %s %s\r\n"
                     "4: %s %s\r\n",
                     c_str[0], s_str[0], c_str[1], s_str[1], c_str[2],
                     s_str[2], c_str[3], s_str[3]);
  extern void BSP_BT_SendPacket(uint8_t *data, uint16_t len);
  BSP_BT_SendPacket((uint8_t *)print_buf, (uint16_t)len);
}

static void _maybe_notify_inspect_end(void) {
  if (s_inspect_end_sent) {
    return;
  }
  if (s_targets_recognized < 4) {
    return;
  }
  if (s_graph_curr_node != NODE_A) {
    return;
  }

  int next_node = _graph_get_neighbor(s_graph_curr_node, s_graph_heading);
  if (next_node == NODE_O) {
    Vision_SendInspectDone();
    s_inspect_end_sent = 1;
  }
}

static void _begin_final_parking(float yaw) {
  if (s_step != HEX_FINAL_PARKING) {
    _bt_print_targets();

    s_target_yaw = yaw;
    _snap_dist();
    MODE_Straight_Init();
    s_step = HEX_FINAL_PARKING;
  }

  /* 立即用直行模式覆盖循迹输出，避免在 O 点大黑块上画龙 */
  MODE_Straight_Exec(HEX_FINAL_PARK_SPEED, s_target_yaw, yaw);
}

static int s_ui_mode = 0;
static int s_ui_timer_ms = 0;

/* ======================== 公共接口 ======================== */

void TaskHex_Init(float start_yaw) {
  s_step = HEX_START_BLIND;
  s_done = 0;
  s_target_yaw = start_yaw;
  s_node_id = 1;
  s_inspect_end_sent = 0;
  s_current_blind_dist = 180.0f;

  s_graph_curr_node = NODE_O;
  s_graph_heading = 3;
  s_targets_recognized = 0;
  s_vision_cleared = 0;
  s_ui_mode = 0;
  s_ui_timer_ms = 0;

  s_cached_vis_turn = -1;
  s_cached_color = -1;
  s_cached_shape = -1;
  for (int i = 0; i < 4; i++) {
    s_recognized_colors[i] = -1;
    s_recognized_shapes[i] = -1;
  }

  DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0); // PA0
  DL_GPIO_clearPins(GPIOB,
                    DL_GPIO_PIN_26 | DL_GPIO_PIN_27); // PB26(Red), PB27(Green)
  DL_GPIO_clearPins(GPIOB,
                    DL_GPIO_PIN_24); // PB24 Buzzer (高电平响，低电平静音)

  Broker_Publish_Int(TOPIC_VISION_TURN, -1);
  Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
  Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);

  _snap_dist();
  MODE_Straight_Init();
}

void TaskHex_SetNextTurn(float turn_angle) { /* 留空，兼容旧接口 */ }

void TaskHex_Exec(void) {
  if (s_done)
    return;

  /* ====== 1ms UI 状态处理 ====== */
  if (s_ui_mode != 0) {
    s_ui_timer_ms++;
    if (s_ui_mode == 1) { /* 红色：蜂鸣器常响 3s，红灯保持亮 */
      DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);    // PA0 红灯(LED1)
      DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_26);   // PB26 红灯(LED2)
      DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_27); // PB27 绿灯(LED2)灭
      if (s_ui_timer_ms <= 3000) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_24); // 高电平响
      } else {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24); // 低电平静音
      }
    } else if (s_ui_mode == 2) { /* 绿色：蜂鸣器断续响 3s，绿灯保持亮 */
      DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);  // PA0 灭
      DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_26); // PB26 灭
      DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_27);   // PB27 绿灯(LED2)亮
      if (s_ui_timer_ms <= 3000) {
        if ((s_ui_timer_ms % 200) < 100) {
          DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_24); // 高电平响
        } else {
          DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24); // 低电平静音
        }
      } else {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24);
      }
    }
  } else {
    /* UI Mode 0：无标识，熄灭一切 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_26 | DL_GPIO_PIN_27);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24);
  }

  float yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
  float track_err = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);

  /* 读取最外侧探头，用于单边分叉检测 (必须严格 >0.60 避免跑偏误触发) */
  float norm_l = Track_GetNormalized(&g_track, 0);
  float norm_r = Track_GetNormalized(&g_track, 7);

  uint8_t strong_l = (norm_l > 0.70f);
  uint8_t strong_r = (norm_r > 0.70f);

  float dist = _get_dist();

  int target_node = -1;
  for (int i = 0; i < 3; i++) {
    if (hex_graph[s_graph_curr_node].heading[i] == s_graph_heading) {
      target_node = hex_graph[s_graph_curr_node].neighbor[i];
      break;
    }
  }

  /* 【抗全局光线/阴影干扰过滤 & 终点停车判定】
     正常赛道(宽2cm)最多只能同时覆盖 2 个探头。
     如果中间 4
     个探头全黑，说明遇到了大面积阴影、发车区黑胶布、或环境光剧变导致的整体漂移！
     此时强制屏蔽外侧探头的假触发，防止直走时画龙。 */
  if (s_step != HEX_FINAL_PARKING) {
    if (strong_l || strong_r) {
      float n2 = Track_GetNormalized(&g_track, 2);
      float n3 = Track_GetNormalized(&g_track, 3);
      float n4 = Track_GetNormalized(&g_track, 4);
      float n5 = Track_GetNormalized(&g_track, 5);
      if (n2 > 0.40f && n3 > 0.40f && n4 > 0.40f && n5 > 0.40f) {
        /* 如果即将回到 O 点，且遇到了大黑块（O点是个大黑块），进入终点加走停车流程 */
        if (target_node == NODE_O && s_targets_recognized >= 4) {
          _begin_final_parking(yaw);
          return;
        }

        strong_l = 0;
        strong_r = 0;
      }
    }
  }

  switch (s_step) {
  case HEX_START_BLIND: {
    MODE_Straight_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);
    /* 脱离O点发车区黑块，走120mm后遇到黑线无缝切入循迹 */
    float n3 = Track_GetNormalized(&g_track, 3);
    float n4 = Track_GetNormalized(&g_track, 4);
    if (dist > 120.0f && (n3 > 0.40f || n4 > 0.40f)) {
      _snap_dist();
      MODE_Tracking_Init();
      s_step = HEX_TRACKING;
    }
    break;
  }

  /* ===== 1. 正常循迹 ===== */
  case HEX_TRACKING: {
    /* 正常循迹执行 */
    MODE_Tracking_Exec(TASK_TRACK_HIGH, track_err);

    /* 缓存任何时刻出现的有效视觉信号，防止路口盲区丢失 */
    int current_vt = Broker_Subscribe_Int(TOPIC_VISION_TURN);
    int current_vc = Broker_Subscribe_Int(TOPIC_VISION_COLOR);
    int current_vs = Broker_Subscribe_Int(TOPIC_VISION_SHAPE);
    if (current_vt != -1) {
      s_cached_vis_turn = current_vt;
    }
    if (current_vc != -1) {
      s_cached_color = current_vc;
    }
    if (current_vs != -1) {
      s_cached_shape = current_vs;
    }

    /* 使用统一盲走距离：防止刚出路口误触发，同时也短到不会错过下一个路口 */
    if (dist > s_current_blind_dist) {
      if (!s_vision_cleared) {
        Broker_Publish_Int(TOPIC_VISION_TURN, -1);
        Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
        Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);
        s_vision_cleared = 1;
      }

      if (target_node != -1) {
        int left_heading = (s_graph_heading + 1) % 6;
        int right_heading = (s_graph_heading + 5) % 6;
        int has_left = 0, has_right = 0;
        for (int i = 0; i < 3; i++) {
          int h = hex_graph[target_node].heading[i];
          if (h == left_heading)
            has_left = 1;
          if (h == right_heading)
            has_right = 1;
        }

        /* 强制剔除不可能的岔路信号 */
        if (has_left && !has_right) {
          strong_r = 0; /* 强制关闭右侧假信号 */
        } else if (!has_left && has_right) {
          strong_l = 0; /* 强制关闭左侧假信号 */
        }
      }

      /* 【终点停车】完美压到终点线 O点 */
      if (target_node == NODE_O && strong_l && strong_r) {
        if (s_targets_recognized >= 4) {
          _begin_final_parking(yaw);
          return;
        }
      }

      /* 【丢线/漏点寻线补救逻辑】
         1. 如果丢线，且跑够了最小边长(>200mm)，说明滑出赛道。
         2.
         如果一直直走，距离超过了最大理论边长(>300mm)，说明路口被大黑块/阴影完全遮挡（被强行滤除）导致漏判。
         此时绝不继续直走（由于六边形不可能有两条连在一起的直线），直接强制按照拓扑图转弯补救！
       */
      if ((track_err == 999.0f && dist > 200.0f) || (dist > 300.0f)) {
        if (target_node != -1 && target_node != NODE_O) {
          int left_heading = (s_graph_heading + 1) % 6;
          int right_heading = (s_graph_heading + 5) % 6;
          int has_left = 0, has_right = 0;
          for (int i = 0; i < 3; i++) {
            int h = hex_graph[target_node].heading[i];
            if (h == left_heading)
              has_left = 1;
            if (h == right_heading)
              has_right = 1;
          }

          int vis_color = s_cached_color;
          int vis_shape = s_cached_shape;
          float turn_angle = 60.0f; /* 默认左转 */

          if (has_left && !has_right) {
            /* 单边拐角，必须左转，不消耗视觉目标，不触发 UI */
            turn_angle = 60.0f;
            s_ui_mode = 0;
          } else if (!has_left && has_right) {
            /* 单边拐角，必须右转，不消耗视觉目标，不触发 UI */
            turn_angle = -60.0f;
            s_ui_mode = 0;
          } else {
            /* 真实的 Y 型路口 */
            if (vis_color == 1) { // 红色 -> 右转
              turn_angle = -60.0f;
              s_ui_mode = 1;
              s_ui_timer_ms = 0;
            } else if (vis_color == 0) { // 绿色 -> 左转
              turn_angle = 60.0f;
              s_ui_mode = 2;
              s_ui_timer_ms = 0;
            } else {
              /* 无有效视觉，按最短路径回 O */
              int best_heading = left_heading;
              int min_dist = 999;
              for (int i = 0; i < 3; i++) {
                if (hex_graph[target_node].heading[i] == left_heading) {
                  int n = hex_graph[target_node].neighbor[i];
                  if (dist_to_O[n] < min_dist) {
                    min_dist = dist_to_O[n];
                    best_heading = left_heading;
                  }
                }
                if (hex_graph[target_node].heading[i] == right_heading) {
                  int n = hex_graph[target_node].neighbor[i];
                  if (dist_to_O[n] < min_dist) {
                    min_dist = dist_to_O[n];
                    best_heading = right_heading;
                  }
                }
              }
              turn_angle = (best_heading == left_heading) ? 60.0f : -60.0f;
              s_ui_mode = 0;
            }

            if (s_targets_recognized < 4) {
              /* 无论是成功识别还是漏识别，真实的 Y 型路口都必须消耗一次目标计数，防止无限绕圈 */
              s_recognized_colors[s_targets_recognized] = vis_color;
              s_recognized_shapes[s_targets_recognized] = vis_shape;
              s_targets_recognized++;
            }
          }

          s_target_yaw = _normalize(s_target_yaw + turn_angle);
          s_graph_curr_node = target_node;
          if (turn_angle > 0)
            s_graph_heading = (s_graph_heading + 1) % 6;
          else
            s_graph_heading = (s_graph_heading + 5) % 6;

          s_cached_vis_turn = -1;
          s_cached_color = -1;
          s_cached_shape = -1;
          Broker_Publish_Int(TOPIC_VISION_TURN, -1);
          Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
          Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);
          Vision_SendNode(s_node_id);
          s_node_id++;

          /* A->O：通知视觉巡检结束，可更新 TXT 并准备掉电保存 */
          _maybe_notify_inspect_end();

          MODE_Turning_Init();
          s_step = HEX_TURNING;
          break;
        } else if (target_node == NODE_O && dist > 300.0f &&
                   s_targets_recognized >= 4) {
          /* 如果目标是 O 点，且走够了距离，说明冲出终点了，直接停车 */
          Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
          Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);

          /* 串口打印识别结果 */
          const char *c_str[4];
          const char *s_str[4];
          for (int i = 0; i < 4; i++) {
            c_str[i] =
                (s_recognized_colors[i] == 1)
                    ? "Red"
                    : ((s_recognized_colors[i] == 0) ? "Green" : "Unknown");
            s_str[i] =
                (s_recognized_shapes[i] == 1)
                    ? "Circle"
                    : ((s_recognized_shapes[i] == 0) ? "Square" : "Unknown");
          }

          static char print_buf[256];
          int len = snprintf(print_buf, sizeof(print_buf),
                             "Return to O. Targets:\r\n"
                             "1: %s %s\r\n"
                             "2: %s %s\r\n"
                             "3: %s %s\r\n"
                             "4: %s %s\r\n",
                             c_str[0], s_str[0], c_str[1], s_str[1], c_str[2],
                             s_str[2], c_str[3], s_str[3]);
          extern void BSP_BT_SendPacket(uint8_t *data, uint16_t len);
          BSP_BT_SendPacket((uint8_t *)print_buf, (uint16_t)len);

          Mode_SetDone();
          s_done = 1;
          return;
        }
      }

      if (strong_l && strong_r) {
        /* 两边同时触发最外侧探头，完美识别 Y字路口 */
        s_target_yaw = yaw; /* 冻结当前航向 */
        _snap_dist();
        MODE_Straight_Init();
        s_step = HEX_CENTERING;
      } else if (strong_l) {
        /* 左侧单边触发 */
        s_target_yaw = yaw;
        _snap_dist();
        MODE_Straight_Init();
        s_step = HEX_COMPENSATE_L;
      } else if (strong_r) {
        /* 右侧单边触发 */
        s_target_yaw = yaw;
        _snap_dist();
        MODE_Straight_Init();
        s_step = HEX_COMPENSATE_R;
      }
    }
    break;
  }

  /* ===== 2. 单边触发补偿 ===== */
  case HEX_COMPENSATE_L: {
    /* 用直线 PID 锁住航向往前盲走，等待右侧压线 */
    MODE_Straight_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);

    if (strong_r) {
      /* 如果右侧压线时，左侧依然没有脱离(>0.40)，说明这绝对是个真实的 Y
         型路口！
         （去掉了极不稳定的斜率判断，防止因为10ms级别的传感器抖动导致真实Y路口被误判为单弯）
       */
      if (norm_l > 0.40f) {
        _snap_dist();
        s_step = HEX_CENTERING;
      } else {
        /* 左侧已经完全白了，说明纯粹是车身画龙滑到了右侧。切换到右侧补偿 */
        s_step = HEX_COMPENSATE_R;
      }
    } else if (dist > MAX_COMPENSATE_DIST) {
      int left_heading = (s_graph_heading + 1) % 6;
      int right_heading = (s_graph_heading + 5) % 6;
      int has_left = 0, has_right = 0;
      for (int i = 0; i < 3; i++) {
        int h = hex_graph[target_node].heading[i];
        if (h == left_heading) has_left = 1;
        if (h == right_heading) has_right = 1;
      }

      if (has_left && has_right) {
        /* 拓扑上这是 Y 型路口，只是右侧没压线。强制进入 Y 路口处理逻辑 */
        s_step = HEX_CENTERING;
      } else {
        /* 走完了补偿距离右侧还没压线，证明这是个单纯向左的转角 */
        s_target_yaw = _normalize(
            s_target_yaw + 60.0f); /* 单一左拐，固定 +60°，不影响 Y 循环变量 */
        s_graph_curr_node = target_node;
        s_graph_heading = (s_graph_heading + 1) % 6; // 左转

        s_cached_vis_turn = -1;
        s_cached_color = -1;
        s_cached_shape = -1;
        Broker_Publish_Int(TOPIC_VISION_TURN, -1);
        Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
        Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);
        Vision_SendNode(s_node_id);
        s_node_id++;

        /* A->O：通知视觉巡检结束，可更新 TXT 并准备掉电保存 */
        _maybe_notify_inspect_end();

        MODE_Turning_Init();
        s_step = HEX_TURNING;
      }
    }
    break;
  }

  case HEX_COMPENSATE_R: {
    MODE_Straight_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);

    if (strong_l) {
      if (norm_r > 0.40f) {
        /* 右侧依然没有脱离(>0.40)，真实的 Y 型路口 */
        _snap_dist();
        s_step = HEX_CENTERING;
      } else {
        /* 右侧完全变白，画龙滑脱到左侧，切换补偿 */
        s_step = HEX_COMPENSATE_L;
      }
    } else if (dist > MAX_COMPENSATE_DIST) {
      int left_heading = (s_graph_heading + 1) % 6;
      int right_heading = (s_graph_heading + 5) % 6;
      int has_left = 0, has_right = 0;
      for (int i = 0; i < 3; i++) {
        int h = hex_graph[target_node].heading[i];
        if (h == left_heading) has_left = 1;
        if (h == right_heading) has_right = 1;
      }

      if (has_left && has_right) {
        /* 拓扑上这是 Y 型路口，只是左侧没压线。强制进入 Y 路口处理逻辑 */
        s_step = HEX_CENTERING;
      } else {
        /* 走完了补偿距离左侧还没压线，证明这是个单纯向右的转角 */
        s_target_yaw = _normalize(
            s_target_yaw - 60.0f); /* 单一右拐，固定 -60°，不影响 Y 循环变量 */
        s_graph_curr_node = target_node;
        s_graph_heading = (s_graph_heading + 5) % 6; // 右转

        s_cached_vis_turn = -1;
        s_cached_color = -1;
        s_cached_shape = -1;
        Broker_Publish_Int(TOPIC_VISION_TURN, -1);
        Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
        Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);
        Vision_SendNode(s_node_id);
        s_node_id++;

        /* A->O：通知视觉巡检结束，可更新 TXT 并准备掉电保存 */
        _maybe_notify_inspect_end();

        MODE_Turning_Init();
        s_step = HEX_TURNING;
      }
    }
    break;
  }

  /* ===== 3. 双边均触发后，向中心靠拢 (Y型路口或大弯) ===== */
  case HEX_CENTERING: {
    MODE_Straight_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);

    /* 走完偏移距离，让车身旋转中心精准处于路口中央 */
    if (dist > CENTER_OFFSET_DIST) {
      /* 确认是 Y 型路口，使用缓存的视觉返回的转向命令 */
      int vis_color = s_cached_color;
      int vis_shape = s_cached_shape;
      float turn_angle = 60.0f; // 默认左转

      /* 打表计算当前所在物理节点 (准备更新位置) */
      int current_y_node = -1;
      for (int i = 0; i < 3; i++) {
        if (hex_graph[s_graph_curr_node].heading[i] == s_graph_heading) {
          current_y_node = hex_graph[s_graph_curr_node].neighbor[i];
          break;
        }
      }

      /* 首先判断物理上是否是真正的 Y 型岔路口 */
      int left_heading = (s_graph_heading + 1) % 6;
      int right_heading = (s_graph_heading + 5) % 6;
      int has_left = 0, has_right = 0;
      for (int i = 0; i < 3; i++) {
        int h = hex_graph[current_y_node].heading[i];
        if (h == left_heading)
          has_left = 1;
        if (h == right_heading)
          has_right = 1;
      }

      if (has_left && !has_right) {
        /* 单边拐角，强制左转，不消耗视觉目标，静音 */
        turn_angle = 60.0f;
        s_ui_mode = 0;
      } else if (!has_left && has_right) {
        /* 单边拐角，强制右转，不消耗视觉目标，静音 */
        turn_angle = -60.0f;
        s_ui_mode = 0;
      } else {
        /* 真实的 Y 型路口 */
        if (vis_color == 1) { // 红色 (Red) -> 右转 (Right)
          turn_angle = -60.0f;
          s_ui_mode = 1; // 红色，长鸣 3s，保持红灯
          s_ui_timer_ms = 0;
        } else if (vis_color == 0) { // 绿色 (Green) -> 左转 (Left)
          turn_angle = 60.0f;
          s_ui_mode = 2; // 绿色，断续响 3s，保持绿灯
          s_ui_timer_ms = 0;
        } else {
          /* 无标识，根据拓扑图选择回家最短路径 */
          int best_heading = left_heading;
          int min_dist = 999;
          for (int i = 0; i < 3; i++) {
            if (hex_graph[current_y_node].heading[i] == left_heading) {
              int n = hex_graph[current_y_node].neighbor[i];
              if (dist_to_O[n] < min_dist) {
                min_dist = dist_to_O[n];
                best_heading = left_heading;
              }
            }
            if (hex_graph[current_y_node].heading[i] == right_heading) {
              int n = hex_graph[current_y_node].neighbor[i];
              if (dist_to_O[n] < min_dist) {
                min_dist = dist_to_O[n];
                best_heading = right_heading;
              }
            }
          }
          turn_angle = (best_heading == left_heading) ? 60.0f : -60.0f;
          s_ui_mode = 0; // 无标识，全灭
        }

        if (s_targets_recognized < 4) {
          /* 无论是成功识别还是漏识别，真实的 Y 型路口都必须消耗一次目标计数，防止无限绕圈 */
          s_recognized_colors[s_targets_recognized] = vis_color;
          s_recognized_shapes[s_targets_recognized] = vis_shape;
          s_targets_recognized++;
        }
      }

      s_target_yaw = _normalize(s_target_yaw + turn_angle);

      /* 更新位置节点 */
      s_graph_curr_node = current_y_node;
      if (turn_angle > 0) {
        s_graph_heading = (s_graph_heading + 1) % 6; // 左转
      } else if (turn_angle < 0) {
        s_graph_heading = (s_graph_heading + 5) % 6; // 右转
      }

      /* 【数据生命周期管理】：消费掉本次视觉指令 */
      s_cached_vis_turn = -1;
      s_cached_color = -1;
      s_cached_shape = -1;
      Broker_Publish_Int(TOPIC_VISION_TURN, -1);
      Broker_Publish_Int(TOPIC_VISION_COLOR, -1);
      Broker_Publish_Int(TOPIC_VISION_SHAPE, -1);

      /* 将当前路口编号发给视觉 */
      Vision_SendNode(s_node_id);
      s_node_id++;

      /* A->O：通知视觉巡检结束，可更新 TXT 并准备掉电保存 */
      _maybe_notify_inspect_end();

      MODE_Turning_Init();
      s_step = HEX_TURNING;
    }
    break;
  }

  /* ===== 4. 陀螺仪闭环旋转 ===== */
  case HEX_TURNING: {
    MODE_Turning_Exec(s_target_yaw, yaw);

    if (MODE_Turning_IsDone()) {
      _snap_dist();
      MODE_Straight_Init();
      s_step = HEX_EXITING;
    }
    break;
  }

  /* ===== 5. 旋转后直行脱离路口交汇区 ===== */
  case HEX_EXITING: {
    MODE_Straight_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);

    if (dist > EXIT_BLIND_DIST) {
      /* 拐角处理完毕，进入下一条边 */

      /* 1. 仅通过快照更新相对里程，千万不要清空底层编码器！
         因为 Broker 存在 10ms 异步延迟，清空底层会导致 dist 瞬间跳变到几百！ */
      _snap_dist(); /* 同步更新上层快照 */

      /* 2. 重置PID，清空积分项，并彻底清零红外传感器残留的黑线数据，防止误识别
       */
      MODE_Tracking_Init();
      extern Track_Instance g_track;
      Track_Clear(&g_track);

      /* 3. 设置 180mm 的循迹恢复盲区！(放宽死区)
         光线干扰严重时，加长该死区可以强制车身在直走中段忽略任何伪路口信号。
         180mm + 前面的 150mm = 330mm，几乎跨越整条边，极大降低误判概率。*/
      s_current_blind_dist = 180.0f;
      s_vision_cleared = 0;

      s_step = HEX_TRACKING;
    }
    break;
  }

  /* ===== 6. 最终回到 O 点：加走一段再停车 ===== */
  case HEX_FINAL_PARKING: {
    MODE_Straight_Exec(HEX_FINAL_PARK_SPEED, s_target_yaw, yaw);

    if (dist > HEX_FINAL_PARK_EXTRA_DIST) {
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      Mode_SetDone();
      s_done = 1;
      return;
    }
    break;
  }

  default:
    break;
  }
}

uint8_t TaskHex_IsDone(void) { return s_done; }
