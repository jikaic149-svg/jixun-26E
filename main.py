# vision_main.py
# 运行环境: MaixPy v4 (MaixCam)
# 功能: 巡检小车颜色形状识别 (抗运动模糊 + 矩形直道ROI + 极致零延迟单帧触发 + 完整UI与UDP显隐回传)

import os
import math
import socket
import cv2
import numpy as np
import time as _std_time
from maix import camera, display, image, app, time, touchscreen, uart, pinmap, sys, gpio

# =========================================================
# 【全局参数配置区】
# =========================================================

class CvVisionConfig:
    """视觉识别与空间逻辑配置核心类"""
    
    # 核心物理死区
    DEAD_ZONE_X_RATIO = 0.28        # 左右画面盲区比例
    DEAD_ZONE_Y_TOP_RATIO = 0.10    # 顶部画面盲区比例
    MIN_VALID_SCORE = 30.0          # 综合目标选定分数下限
    
    # 图像分辨率
    FRAME_WIDTH = 320
    FRAME_HEIGHT = 240

    # 颜色 HSV 阈值 (根据标定工具锁定所得精确更新)
    HSV_RED_LOWER1 = np.array([0, 50, 75])
    HSV_RED_UPPER1 = np.array([12, 255, 255])
    HSV_RED_LOWER2 = np.array([160, 50, 75])
    HSV_RED_UPPER2 = np.array([180, 255, 255])

    HSV_GREEN_LOWER = np.array([48, 40, 45])
    HSV_GREEN_UPPER = np.array([78, 255, 255])

    # 掩膜形态学滤波参数
    MASK_KERNEL_SIZE = 5           
    MASK_CLOSE_ITER = 1            

    # 矩形直道有效区域 ROI 配置
    ENABLE_Y_ROI = True            
    Y_TOP_Y_RATIO = 0.28           
    Y_TRUNK_HALF_WIDTH_RATIO = 0.07 # 限制在左右中心各 7% 宽度内（原0.09），收窄ROI通道以屏蔽邻道色块

    # --- 高速形状判断特征优化 ---
    MIN_AREA = 150                 
    SHAPE_MIN_AREA = 150           
    SHAPE_MIN_CY_RATIO = 0.0       
    POLY_EPSILON_RATIO = 0.025      # 调大系数（原0.015），高速下过滤边缘发毛和噪声毛刺
    ELLIPSE_FILL_MIN = 0.90        
    SQUARE_EDGE_RATIO_MAX = 2.5    
    RECT_FILL_SQUARE_MIN = 0.81     # 调大门槛（原0.78），避免圆形（最大约0.785）在边缘抖动时被误判为正方形

    # --- 极致时序响应：单帧触发，零延迟 ---
    FILTER_WINDOW_SIZE = 1          # 1帧缓存，不积压过去数据，防止时序错乱
    CONFIRM_VOTES = 1               # 1票即过，第一时间锁定并输出

    # 打分权重参数
    MAX_EXPECTED_AREA = 4000.0     
    WEIGHT_AREA = 0.3              
    WEIGHT_CENTER = 0.7            


# 系统与调试参数
DEBUG_MODE = False                           
HISTORY_SAVE_FILE = "/root/inspection_history.txt" 
MAX_HISTORY_LEN = 20                         
RECORD_TARGET_COUNT = 20                     # 巡检需记录的最多目标数量

# 通信参数
UDP_IP = "255.255.255.255"                     # UDP 透传广播IP地址
UDP_PORT = 8080                               # UDP 透传目标端口
UART_DEVICE = "/dev/ttyS0"                   
UART_BAUDRATE = 115200                       
UART_PIN_TX = "A16"                          
UART_PIN_RX = "A17"                          

# --- 高速底层物理曝光优化 ---
CAM_EXPOSURE = 8000                           # 提高到 8000us (8ms)，运动模糊极小但亮度翻倍
CAM_GAIN = 80                                 # 增益提升到 80，配合曝光大幅提亮画面
TURN_END_BLIND_TIME = 0.2                    

# 常量字典与基础色彩
TARGET_CN_MAP = {
    "red_circle": "红圆",
    "red_square": "红方",
    "green_circle": "绿圆",
    "green_square": "绿方",
    "red_unknown": "红块",    
    "green_unknown": "绿块",  
    "none": "无"
}

COLOR_W = image.Color.from_rgb(255, 255, 255)
COLOR_R = image.Color.from_rgb(255, 0, 0)
COLOR_G = image.Color.from_rgb(0, 255, 0)
COLOR_Y = image.Color.from_rgb(255, 255, 0)

# =========================================================
# 【全局外设与网络初始化】
# =========================================================
# 已恢复 UDP 广播外设初始化
try:
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP 透传初始化成功! 将数据广播至 {UDP_IP}:{UDP_PORT}")
except Exception as e:
    print(f"UDP 初始化失败: {e}")
    udp_socket = None

try:
    pinmap.set_pin_function(UART_PIN_TX, "UART0_TX")
    pinmap.set_pin_function(UART_PIN_RX, "UART0_RX")
except Exception as e:
    print(f"引脚映射提示: {e}")

try:
    UART_PORT = uart.UART(UART_DEVICE, UART_BAUDRATE)
    print(f"UART 串口初始化成功! ({UART_DEVICE}, 映射 {UART_PIN_TX}/{UART_PIN_RX})")
except Exception as e:
    print(f"UART 串口初始化失败: {e}")
    UART_PORT = None

# =========================================================
# 系统核心运行代码
# =========================================================

def save_history(queue, inspection_seconds=None):
    try:
        targets_str = ",".join(queue)
        targets_cn = "".join([TARGET_CN_MAP.get(item.strip(), "未知") for item in queue])

        lines = [
            f"targets={targets_str}",
            f"targets_cn={targets_cn}"
        ]

        if inspection_seconds is not None:
            lines.append(f"inspection_time={inspection_seconds:.2f}s")

        data_str = "\n".join(lines)

        with open(HISTORY_SAVE_FILE, "w", encoding="utf-8") as f:
            f.write(data_str)
            f.flush()
            os.fsync(f.fileno())
        print(f"[存储成功] 历史记录已保存:\n{data_str}")
    except Exception as e:
        print(f"[存储错误] 无法保存历史: {e}")

def load_history():
    if os.path.exists(HISTORY_SAVE_FILE):
        try:
            with open(HISTORY_SAVE_FILE, "r", encoding="utf-8") as f:
                data_str = f.read().strip()
                if data_str:
                    for line in data_str.splitlines():
                        if line.startswith("targets="):
                            targets_str = line.split("=", 1)[1].strip()
                            if targets_str:
                                return targets_str.split(",")
                            return []
                    return data_str.split(",")
        except Exception as e:
            print(f"[读取错误] 无法读取历史: {e}")
    return []

def load_last_inspection_seconds():
    if not os.path.exists(HISTORY_SAVE_FILE):
        return 0.0
    try:
        with open(HISTORY_SAVE_FILE, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("inspection_time="):
                    value = line.split("=", 1)[1].strip()
                    value = value.replace("s", "")
                    return float(value)
    except Exception as e:
        print(f"[巡检计时读取错误] {e}")
    return 0.0

class InspectionHistory:
    def __init__(self):
        self.queue = load_history()

    def clear(self):
        self.queue.clear()

    def append_target(self, target_name):
        if not target_name or target_name not in TARGET_CN_MAP:
            return
        self.queue.append(target_name)
        if len(self.queue) > MAX_HISTORY_LEN:
            self.queue.pop(0)

    def commit(self, inspection_seconds=None):
        save_history(self.queue, inspection_seconds)

    def to_cn_string(self):
        if not self.queue:
            return ""
        cn_list = [TARGET_CN_MAP.get(item.strip(), "未知") for item in self.queue]
        return "".join(cn_list)

class CvShapeDetector:
    def __init__(self, config):
        self.cfg = config
        self.roi = []
        self.history_buffer = []

    def _build_straight_roi_mask(self, mask_shape):
        """构造纯矩形直道有效识别区域"""
        h, w = mask_shape[:2]
        roi = np.zeros((h, w), dtype=np.uint8)

        cx = w // 2
        y_top = int(h * self.cfg.Y_TOP_Y_RATIO)
        trunk_half = int(w * self.cfg.Y_TRUNK_HALF_WIDTH_RATIO)

        cv2.rectangle(roi, (cx - trunk_half, y_top), (cx + trunk_half, h), 255, -1)
        return roi

    def _get_color_mask(self, hsv_img, color_type):
        if color_type == "red":
            mask1 = cv2.inRange(hsv_img, self.cfg.HSV_RED_LOWER1, self.cfg.HSV_RED_UPPER1)
            mask2 = cv2.inRange(hsv_img, self.cfg.HSV_RED_LOWER2, self.cfg.HSV_RED_UPPER2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color_type == "green":
            mask = cv2.inRange(hsv_img, self.cfg.HSV_GREEN_LOWER, self.cfg.HSV_GREEN_UPPER)
        else:
            mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)

        h, w = mask.shape[:2]
        dead_x = int(w * self.cfg.DEAD_ZONE_X_RATIO)
        dead_y = int(h * self.cfg.DEAD_ZONE_Y_TOP_RATIO)

        mask[:, :dead_x] = 0
        mask[:, w - dead_x:] = 0
        mask[:dead_y, :] = 0

        if self.cfg.ENABLE_Y_ROI:
            straight_roi = self._build_straight_roi_mask(mask.shape)
            mask = cv2.bitwise_and(mask, straight_roi)

        k = self.cfg.MASK_KERNEL_SIZE
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.cfg.MASK_CLOSE_ITER)

        # 核心增强：填充掩膜内部因强光反光泛白而产生的空洞，恢复实心色块形状
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is not None:
            for i in range(len(contours)):
                cv2.drawContours(mask, contours, i, 255, -1)

        return mask

    def _detect_shape(self, hull, area, perimeter, x, y, w, h):
        cx = x + w // 2
        cy = y + h // 2

        circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0.0
        shape = "unknown"
        vertex_count = 0
        ellipse_fill = 0.0
        rect_fill = 0.0
        edge_ratio = 999.0

        if area < self.cfg.SHAPE_MIN_AREA or cy < self.cfg.FRAME_HEIGHT * self.cfg.SHAPE_MIN_CY_RATIO:
            return shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio

        epsilon = self.cfg.POLY_EPSILON_RATIO * perimeter
        approx = cv2.approxPolyDP(hull, epsilon, True)
        vertex_count = len(approx)

        rect = cv2.minAreaRect(hull)
        rw, rh = rect[1]
        if rw > 0 and rh > 0:
            rect_area = rw * rh
            rect_fill = area / rect_area
            rect_aspect_ratio = max(rw, rh) / min(rw, rh)
        else:
            rect_fill = 0
            rect_aspect_ratio = 999

        if len(hull) >= 5:
            try:
                ellipse = cv2.fitEllipse(hull)
                (_, _), (axis1, axis2), _ = ellipse
                if axis1 > 0 and axis2 > 0:
                    ellipse_area = np.pi * (axis1 / 2.0) * (axis2 / 2.0)
                    ellipse_fill = area / ellipse_area
            except Exception:
                pass

        is_square = False
        if cv2.isContourConvex(approx) and 4 <= vertex_count <= 5:
            pts = approx.reshape(-1, 2)
            edge_lengths = []
            for i in range(vertex_count):
                p1 = pts[i]
                p2 = pts[(i + 1) % vertex_count]
                edge_lengths.append(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))
        
            min_edge = min(edge_lengths) if edge_lengths else 0
            max_edge = max(edge_lengths) if edge_lengths else 0
            edge_ratio = max_edge / min_edge if min_edge > 0 else 999
            
            if (edge_ratio <= self.cfg.SQUARE_EDGE_RATIO_MAX
                and rect_fill >= self.cfg.RECT_FILL_SQUARE_MIN
                and rect_aspect_ratio <= 3.0):
                # 只有当外接椭圆填充率明确偏低（<0.90）或无椭圆时圆度偏低（<0.78）才判定为正方形，防止圆形（这两个值均很高）混入
                if (ellipse_fill > 0.0 and ellipse_fill < 0.90) or (ellipse_fill == 0.0 and circularity < 0.78):
                    is_square = True

        if is_square:
            shape = "square"
        else:
            if vertex_count >= 5 and len(hull) >= 5:
                condition1 = (vertex_count >= 6 and ellipse_fill >= self.cfg.ELLIPSE_FILL_MIN)
                condition2 = (vertex_count == 5 and ellipse_fill >= 0.90)
                if condition1 or condition2:
                    shape = "circle"

        if shape == "unknown":
            if ellipse_fill > 0.85 or circularity > 0.78:
                shape = "circle"
            elif rect_fill > 0.82:
                shape = "square"
            else:
                shape = "square"

        return shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio

    def _select_best_target(self, current_frame_results):
        best_target_name = "none"
        highest_score = -1.0
        center_x = self.cfg.FRAME_WIDTH / 2.0

        for res in current_frame_results:
            if res.get("shape") == "discarded":
                continue

            cx = res["cx"]
            area = res["area"]
            color = res["color"]
            shape = res["shape"]

            score_area = min(100.0, (area / self.cfg.MAX_EXPECTED_AREA) * 100.0)
            dist_to_center = abs(cx - center_x)
            score_center = max(0.0, 100.0 - (dist_to_center / center_x) * 100.0)

            total_score = (score_area * self.cfg.WEIGHT_AREA + score_center * self.cfg.WEIGHT_CENTER)

            if DEBUG_MODE:
                print(f"[{color}_{shape}] area={area:.1f} score_area={score_area:.1f} score_center={score_center:.1f} total={total_score:.1f}")

            if total_score > highest_score and total_score >= self.cfg.MIN_VALID_SCORE:
                highest_score = total_score
                best_target_name = f"{color}_{shape}"

        return best_target_name

    def process_frame(self, img_maix, history_len=0):
        img_cv = image.image2cv(img_maix, copy=False)
        hsv_img = cv2.cvtColor(img_cv, cv2.COLOR_RGB2HSV)
        current_frame_results = []

        for color_name in ["red", "green"]:
            mask = self._get_color_mask(hsv_img, color_name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < self.cfg.MIN_AREA:
                    continue

                hull = cv2.convexHull(cnt)
                area = cv2.contourArea(hull)
                perimeter = cv2.arcLength(hull, True)

                if area < self.cfg.MIN_AREA or perimeter <= 0:
                    continue

                x, y, w, h = cv2.boundingRect(hull)
                cx = x + w // 2
                cy = y + h // 2

                dead_zone_margin_x = self.cfg.FRAME_WIDTH * self.cfg.DEAD_ZONE_X_RATIO
                dead_zone_margin_y_top = self.cfg.FRAME_HEIGHT * self.cfg.DEAD_ZONE_Y_TOP_RATIO

                discard_reason = ""
                if cx < dead_zone_margin_x or cx > self.cfg.FRAME_WIDTH - dead_zone_margin_x:
                    discard_reason = "X边缘"
                elif cy < dead_zone_margin_y_top:
                    discard_reason = "Y顶部"

                if discard_reason:
                    current_frame_results.append({
                        "color": color_name, "shape": "discarded",
                        "x": x, "y": y, "w": w, "h": h, "cx": cx, "cy": cy,
                        "area": area, "reason": discard_reason
                    })
                    continue

                shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio = self._detect_shape(
                    hull, area, perimeter, x, y, w, h
                )

                current_frame_results.append({
                    "color": color_name, "shape": shape,
                    "x": x, "y": y, "w": w, "h": h, "cx": cx, "cy": cy, "area": area,
                    "roundness": circularity, "vertex": vertex_count,
                    "ellipse_fill": ellipse_fill, "rect_fill": rect_fill, "edge_ratio": edge_ratio
                })

        target_name = self._select_best_target(current_frame_results)
        self.history_buffer.append(target_name)

        if len(self.history_buffer) > self.cfg.FILTER_WINDOW_SIZE:
            self.history_buffer.pop(0)

        stable_result = None
        valid_targets = [t for t in self.history_buffer if t != "none"]
        if valid_targets:
            best_target = max(set(valid_targets), key=valid_targets.count)
            if valid_targets.count(best_target) >= self.cfg.CONFIRM_VOTES:
                stable_result = best_target
                
        if stable_result is None and len(self.history_buffer) == self.cfg.FILTER_WINDOW_SIZE:
            stable_result = "none"

        return current_frame_results, stable_result

    def draw_debug_info(self, img_maix, results, stable_result):
        dead_zone_margin_x = int(self.cfg.FRAME_WIDTH * self.cfg.DEAD_ZONE_X_RATIO)
        img_maix.draw_line(dead_zone_margin_x, 0, dead_zone_margin_x, self.cfg.FRAME_HEIGHT, COLOR_R, 1)
        img_maix.draw_line(self.cfg.FRAME_WIDTH - dead_zone_margin_x, 0, self.cfg.FRAME_WIDTH - dead_zone_margin_x, self.cfg.FRAME_HEIGHT, COLOR_R, 1)

        dead_zone_margin_y_top = int(self.cfg.FRAME_HEIGHT * self.cfg.DEAD_ZONE_Y_TOP_RATIO)
        img_maix.draw_line(0, dead_zone_margin_y_top, self.cfg.FRAME_WIDTH, dead_zone_margin_y_top, COLOR_R, 1)

        shape_min_y = int(self.cfg.FRAME_HEIGHT * self.cfg.SHAPE_MIN_CY_RATIO)
        img_maix.draw_line(0, shape_min_y, self.cfg.FRAME_WIDTH, shape_min_y, COLOR_Y, 1)

        if self.cfg.ENABLE_Y_ROI:
            cx = self.cfg.FRAME_WIDTH // 2
            y_top = int(self.cfg.FRAME_HEIGHT * self.cfg.Y_TOP_Y_RATIO)
            trunk_half = int(self.cfg.FRAME_WIDTH * self.cfg.Y_TRUNK_HALF_WIDTH_RATIO)
            
            img_maix.draw_rect(cx - trunk_half, y_top, trunk_half * 2, self.cfg.FRAME_HEIGHT - y_top, COLOR_Y, 2)
            img_maix.draw_line(0, y_top, self.cfg.FRAME_WIDTH, y_top, COLOR_Y, 2)

        for res in results:
            if res.get("shape") == "discarded":
                img_maix.draw_rect(res["x"], res["y"], res["w"], res["h"], COLOR_W, 1)
                img_maix.draw_string(res["x"], max(0, res["y"] - 15), f"屏蔽:{res['reason']}", COLOR_W)
                continue

            rect_color = COLOR_R if res["color"] == "red" else COLOR_G
            img_maix.draw_rect(res["x"], res["y"], res["w"], res["h"], rect_color, 2)
            img_maix.draw_cross(res["cx"], res["cy"], COLOR_W, 5)

            label = f"{res['shape']} A:{res['area']:.0f}"
            img_maix.draw_string(res["x"], max(0, res["y"] - 20), label, rect_color)

        if stable_result:
            img_maix.draw_string(2, 210, f"STABLE: {stable_result}", color=COLOR_W, scale=1.5)


# =========================================================
# UART 通信协议
# =========================================================
def send_uart_command(uart_port, target_name):
    if target_name is None or "_" not in target_name:
        return False

    color_str, shape_str = target_name.split("_")
    if color_str not in ["red", "green"]:
        return False

    val_color = 0x01 if color_str == "red" else 0x00
    val_turn = 0x01 if color_str == "red" else 0x00

    if shape_str == "circle":
        val_shape = 0x01
    elif shape_str == "square":
        val_shape = 0x00
    else:
        val_shape = 0xFF

    checksum = (val_turn + val_color + val_shape) & 0xFF
    packet = bytearray([0xAA, 0x55, val_turn, val_color, val_shape, checksum])

    if uart_port:
        try:
            uart_port.write(bytes(packet))
            return True
        except Exception as e:
            print(f"UART 发送失败: {e}")
    return False


class InspectionCarApp:
    def __init__(self):
        self.cam = camera.Camera(320, 240)
        self.disp = display.Display()
        self.history = InspectionHistory()
        self.config = CvVisionConfig()
        self.detector = CvShapeDetector(self.config)
        self._uart_turn_event = False
        self.current_target = None
        self.pending_target = None
        self.display_cn_target = "无"
        self.btn_x, self.btn_y, self.btn_w, self.btn_h = 80, 180, 160, 40
        self.uart_rx_text = ""
        self.history_committed = False
        self.inspect_start_time = None
        self.inspect_end_time = None
        self.uart_send_locked = False  
        self.last_turn_was_y = False   
        self.inspect_display_seconds = load_last_inspection_seconds()
        self.last_sent_target = None  
        self.last_sent_color = None  
        self.color_stable_frames = 0  
        self.is_running_silent = False
        self.started = False                   
        self.last_turn_was_fallback = False    
        self._init_fill_light()

    def reset_run_states(self):
        """完全重置所有运行状态与临时变量，确保新的一轮发车完全干净"""
        self.history.clear()
        self.pending_target = None
        self.current_target = "none"
        self.last_sent_color = None
        self.last_sent_target = None
        self.color_stable_frames = 0
        self.history_committed = False
        self.inspect_end_time = None
        self.inspect_display_seconds = 0.0
        self.is_turning = False
        self.uart_send_locked = False
        self.last_turn_was_fallback = False
        self.last_turn_was_y = False
        self._uart_turn_event = False
        self.started = False
        if hasattr(self, 'detector') and self.detector:
            self.detector.history_buffer.clear()
        print("[状态重置] 所有运行状态、防抖及历史缓存已完全清空重置")

    def __del__(self):
        if getattr(self, "fill_light", None):
            try:
                self.fill_light.value(0)
            except Exception:
                pass

    def _init_fill_light(self):
        try:
            device = sys.device_id()
            if device == "maixcam2":
                pin_name = "B25"
                gpio_name = "GPIOB25"
            else:
                pin_name = "B3"
                gpio_name = "GPIOB3"
            
            pinmap.set_pin_function(pin_name, gpio_name)
            self.fill_light = gpio.GPIO(gpio_name, gpio.Mode.OUT)
            self.fill_light.value(1)
            print(f"[硬件] 开启补光灯成功 (引脚: {pin_name})")
        except Exception as e:
            self.fill_light = None
            print(f"[硬件] 开启补光灯失败: {e}")

    def _load_font(self):
        try:
            image.load_font("sourcehansans", "/maixapp/share/font/SourceHanSansCN-Regular.otf")
            image.set_default_font("sourcehansans")
        except Exception:
            pass

    def _review_history_until_start(self):
        ts = touchscreen.TouchScreen()
        review_mode = True

        while review_mode and not app.need_exit():
            img_maix = self.cam.read()
            if img_maix is None:
                continue

            img_maix.draw_rect(0, 0, 320, 240, image.Color.from_rgb(0, 0, 0), -1)
            img_maix.draw_string(20, 30, "上一次巡检记录审查", COLOR_W, scale=1.5)

            if not self.history.queue:
                img_maix.draw_string(20, 90, "暂无历史数据", COLOR_W)
            else:
                hist_str = self.history.to_cn_string()
                img_maix.draw_string(10, 90, hist_str, COLOR_W)

            img_maix.draw_rect(self.btn_x, self.btn_y, self.btn_w, self.btn_h, COLOR_W, 2)
            img_maix.draw_string(self.btn_x + 20, self.btn_y + 10, "点击开始发车", COLOR_W)

            self._draw_inspection_time(img_maix)
            self.disp.show(img_maix)

            try:
                x, y, pressed = ts.read()
                if pressed:
                    review_mode = False
                    self.reset_run_states()
                    self.started = True
                    self.inspect_start_time = _std_time.time()
                    print("[巡检计时] 点击开始发车，计时清零并开始")
                    
                    if UART_PORT:  
                        UART_PORT.write(b"INSPECT_START\n")
            except Exception:
                pass
        del ts

    def _configure_camera(self):
        self.cam.exp_mode(camera.AeMode.Manual)
        self.cam.exposure(CAM_EXPOSURE)
        self.cam.gain(CAM_GAIN)

    def _commit_history_with_time(self):
        if self.history_committed:
            return
        if self.inspect_start_time is None:
            self.inspect_start_time = _std_time.time()

        self.inspect_end_time = _std_time.time()
        inspection_seconds = self.inspect_end_time - self.inspect_start_time
        self.inspect_display_seconds = inspection_seconds

        self.history.commit(inspection_seconds)
        self.history_committed = True
        print(f"[巡检计时] 巡检用时: {inspection_seconds:.2f}s，已锁定")

        if UART_PORT:
            try:
                targets_cn = self.history.to_cn_string()
                replay_msg = f"Path: {targets_cn}, Time: {inspection_seconds:.2f}s\r\n"
                UART_PORT.write(replay_msg.encode("utf-8"))
            except Exception as e:
                print(f"[UART 异常] 回放回传失败: {e}")

    def _get_inspection_elapsed_seconds(self):
        if self.inspect_start_time is None:
            return self.inspect_display_seconds
        if self.history_committed:
            return self.inspect_display_seconds
        return _std_time.time() - self.inspect_start_time

    def _draw_inspection_time(self, img_maix):
        elapsed = self._get_inspection_elapsed_seconds()
        img_maix.draw_string(210, 218, f"T:{elapsed:.2f}s", COLOR_W)

    def _process_uart_input(self):
        if not hasattr(self, 'uart_rx_buffer'):
            self.uart_rx_buffer = bytearray()
            self.last_uart_print_time = 0.0

        if UART_PORT:
            try:
                in_data = UART_PORT.read()
                if in_data:
                    self.uart_rx_buffer.extend(in_data)
                    
                    while b'\n' in self.uart_rx_buffer:
                        idx = self.uart_rx_buffer.index(b'\n')
                        line_data = self.uart_rx_buffer[:idx+1]
                        self.uart_rx_buffer = self.uart_rx_buffer[idx+1:]
                       
                        try:
                            text = line_data.decode("utf-8", "ignore")
                            clean_text = text.strip()
                            
                            vis_target = self.current_target if getattr(self, 'current_target', None) else "none"
                            cur_fps = time.fps()
                            
                            current_time = _std_time.time()
                            if clean_text and (current_time - self.last_uart_print_time > 1.0):
                                print(f"[UART RX {cur_fps:.1f}fps] {clean_text} | Vis:{vis_target}")
                                self.last_uart_print_time = current_time

                            if text.startswith("TURN_DECISION:"):
                                dec_str = text.strip()
                                if "VisRead=-1" in dec_str:
                                    self.last_turn_was_fallback = True
                                else:
                                    self.last_turn_was_fallback = False
                                continue

                            self.uart_rx_text += text
                       
                            if "TURN_START\n" in self.uart_rx_text:
                                self.is_turning = True
                                self._uart_turn_event = True 
                                self.last_turn_was_y = True
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_START\n", "")
                        
                                sent_color = getattr(self, "last_sent_target", "无(未锁定)")
                                print(f"\n==== [单片机已切弯] 收到 TURN_START！视觉立即闭眼！上一个发送颜色: {sent_color} ====")

                            elif "TURN_BLIND_ONLY\n" in self.uart_rx_text:
                                self.is_turning = True 
                                self.last_turn_was_y = False
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_BLIND_ONLY\n", "")
                                print("[视觉防抖] 收到 TURN_BLIND_ONLY (L字拐角)，进入闭眼保护")

                            elif "TURN_END\n" in self.uart_rx_text:
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_END\n", "")
                                self.is_turning = False
                                self.uart_send_locked = False
                                self.turn_end_time = _std_time.time()

                                if self.last_turn_was_y:
                                    self.detector.history_buffer.clear()
                                    self.current_target = None
                                    self.pending_target = None
                                    self.display_cn_target = "无"
                                    self.last_sent_color = None
                                    self.last_sent_target = None
                                    self.color_stable_frames = 0
                                    print("[视觉恢复] Y 路口 TURN_END，全量清空视觉锁")
                                else:
                                    self.last_sent_color = None
                                    self.last_sent_target = None
                                    print("[视觉恢复] L 拐角 TURN_END，重置颜色强制重发")
                            
                            if "INSPECT_START\n" in self.uart_rx_text:
                                print(f"[视觉复位] 收到单片机发车信号 INSPECT_START")
                                self.reset_run_states()
                                self.started = True
                                self.inspect_start_time = _std_time.time()
                                self.uart_rx_text = self.uart_rx_text.replace("INSPECT_START\n", "")

                            if ("INSPECT_END" in self.uart_rx_text or "end\n" in self.uart_rx_text.lower()) and not self.history_committed:
                                print(f"[巡检完成] 收到单片机 end 信号")
                                self._commit_history_with_time()
                                self.uart_rx_text = self.uart_rx_text.replace("INSPECT_END", "").replace("end\n", "").replace("END\n", "")

                            if len(self.uart_rx_text) > 64:
                                self.uart_rx_text = self.uart_rx_text[-64:]
                        except Exception:
                            pass
            except Exception:
                pass

    def _record_pending_target_by_turn(self):
        if self.history_committed:
            return
        if len(self.history.queue) >= RECORD_TARGET_COUNT:
            return
        if not self.pending_target:
            return

        self.history.append_target(self.pending_target)
        print(f"[历史记录] 角度突变确认，第 {len(self.history.queue)} 次录入: {self.pending_target}")
        self.history.commit()
        
        confirmed_str = TARGET_CN_MAP.get(self.pending_target, "无")
        self.display_cn_target = f"转弯点亮: {confirmed_str}"
        self.pending_target = None

    def _handle_stable_result(self, stable_result, turn_event):
        if not self.started:
            self.current_target = None
            self.pending_target = None
            self.color_stable_frames = 0
            self.display_cn_target = "等待发车"
            return

        if turn_event:
            if getattr(self, "last_turn_was_fallback", False):
                print("[防抖保护] 单片机盲猜回退，拒绝录入当前目标")
            elif self.current_target and self.current_target != "none":
                self.pending_target = self.current_target
                self._record_pending_target_by_turn()

            self.is_turning = True 
            self.current_target = None
            self.display_cn_target = "转弯盲区"
            self.uart_send_locked = True
            self.last_sent_color = None  
            self.color_stable_frames = 0
            self.detector.history_buffer.clear()
            return

        if getattr(self, "is_turning", False) or (_std_time.time() - getattr(self, "turn_end_time", 0) < TURN_END_BLIND_TIME):
            return

        if stable_result and stable_result != "none":
            if not self.history_committed:
                color = stable_result.split("_")[0]
                if color != self.last_sent_color:
                    self.last_sent_color = color
                    self.color_stable_frames = 0
                    self.last_sent_target = None  

                self.color_stable_frames += 1

                should_send = False
                if (self.color_stable_frames >= 1 and
                    self.last_sent_target != stable_result and
                    not self.uart_send_locked):
                    if self.last_sent_target is None:
                        should_send = True
                    else:
                        prev_shape = self.last_sent_target.split("_", 1)[1] if "_" in self.last_sent_target else ""
                        new_shape = stable_result.split("_", 1)[1] if "_" in stable_result else ""
                        is_downgrade = (prev_shape in ("circle", "square") and new_shape == "unknown")
                        should_send = not is_downgrade

                if should_send:
                    send_uart_command(UART_PORT, stable_result)
                    self.last_sent_target = stable_result
                    print(f"\n>>>> [视觉发送] 耗时 {self.color_stable_frames} 帧，目标: {stable_result} 已发送 <<<<\n")

                self.current_target = stable_result
                self.pending_target = stable_result
                target_str = TARGET_CN_MAP.get(self.current_target, "无")
                self.display_cn_target = f"已锁定待转: {target_str}"

        if stable_result == "none" or stable_result is None:
            self.current_target = None
            self.color_stable_frames = 0
            if not self.pending_target:
                self.display_cn_target = "无"
            if not turn_event:
                self.uart_send_locked = False

    def run(self):
        self._load_font()
        self._configure_camera()
        print("视觉系统完整配置初始化完成")

        while not app.need_exit():
            # 1. 每次循环开始，重新从 TXT 载入最新历史记录和耗时
            self.history.queue = load_history()
            self.inspect_display_seconds = load_last_inspection_seconds()
            
            # 进入等待界面（会高频刷新屏幕显示上一次的记录，直到点击屏幕发车）
            self._review_history_until_start()
            
            # 退出上级等待界面说明已点击屏幕发车。此时进入“发车过渡期”（is_running_silent = False）
            # 屏幕继续刷新摄像头实时画面，直到单片机真正启动并回复 INSPECT_START 才关闭刷新进入静默
            self.is_running_silent = False
            print("[系统状态] 已点击发车，屏幕持续刷新摄像头画面，等待单片机启动...")
            
            # 2. 运行内循环，直到收到单片机 end 信号 (history_committed 变为 True)
            while not self.history_committed and not app.need_exit():
                turn_event = self._uart_turn_event
                self._uart_turn_event = False

                img_maix = self.cam.read()
                if img_maix is None:
                    continue
                self._process_uart_input()

                # 若已收到结束信号，立即退出内循环
                if self.history_committed:
                    break

                history_len = len(self.history.queue)
                results, stable_result = self.detector.process_frame(img_maix, history_len)
                
                # 只有在未静默时，才渲染 UI 并刷新屏幕，单片机启动后（is_running_silent = True）则关闭刷新
                if not self.is_running_silent:
                    self.detector.draw_debug_info(img_maix, results, stable_result)
                    img_maix.draw_string(2, 2, f"FPS: {time.fps():.1f}", color=COLOR_W)
                    img_maix.draw_rect(80, 0, 240, 25, image.Color.from_rgb(0, 0, 0), -1)
                    img_maix.draw_string(85, 2, f"实时: {self.display_cn_target}", COLOR_W)
                    self._draw_inspection_time(img_maix)
                    self.disp.show(img_maix)

                self._handle_stable_result(stable_result, turn_event)
                
            print("[系统状态] 巡检已结束，保存数据并重载展示结果。")


def main():
    inspection_app = InspectionCarApp()
    inspection_app.run()


if __name__ == "__main__":
    main()