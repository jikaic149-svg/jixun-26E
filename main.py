# vision_main.py
# 运行环境: MaixPy v4 (MaixCam)
# 功能: 巡检小车颜色形状识别 (凸包抗反光 + 启发式空间打分 + 稳定单片机业务流 + 历史轨迹审查与触控发车)
# 修改说明: 最小改动版，扩大红/绿暗光识别范围，并通过 Y 字 ROI 尽量滤除赛道外光线干扰
from maix import camera, display, image, app, time
import cv2
import numpy as np
import math
import os
import time as _std_time
from maix import touchscreen


COLOR_W = image.Color.from_rgb(255, 255, 255)
COLOR_R = image.Color.from_rgb(255, 0, 0)
COLOR_G = image.Color.from_rgb(0, 255, 0)
COLOR_Y = image.Color.from_rgb(255, 255, 0)

# =========================================================
# 历史轨迹记录核心模块
# =========================================================
HISTORY_SAVE_FILE = "/root/inspection_history.txt"
MAX_HISTORY_LEN = 20
RECORD_TARGET_COUNT = 4

TARGET_CN_MAP = {
    "red_circle": "红圆",
    "red_square": "红方",
    "green_circle": "绿圆",
    "green_square": "绿方",
    "red_unknown": "红块",    # 新增：即使远处看不清形状，也要作为有效目标录入
    "green_unknown": "绿块",  # 新增
    "none": "无"
}


def save_history(queue, inspection_seconds=None):
    try:
        targets_str = ",".join(queue)
        targets_cn = " -> ".join([TARGET_CN_MAP.get(item.strip(), "未知") for item in queue])

        lines = [
            f"targets={targets_str}",
            f"targets_cn={targets_cn}"
        ]

        if inspection_seconds is not None:
            lines.append(f"inspection_time={inspection_seconds:.2f}s")

        data_str = "\n".join(lines)

        with open(HISTORY_SAVE_FILE, "w") as f:
            f.write(data_str)
            f.flush()
            os.fsync(f.fileno())

        print(f"[存储成功] 历史记录已保存:\n{data_str}")
    except Exception as e:
        print(f"[存储错误] 无法保存历史: {e}")


def load_history():
    if os.path.exists(HISTORY_SAVE_FILE):
        try:
            with open(HISTORY_SAVE_FILE, "r") as f:
                data_str = f.read().strip()
                if data_str:
                    # 新格式：targets=red_circle,green_square,...
                    for line in data_str.splitlines():
                        if line.startswith("targets="):
                            targets_str = line.split("=", 1)[1].strip()
                            if targets_str:
                                return targets_str.split(",")
                            return []

                    # 兼容旧格式：red_circle,green_square,...
                    return data_str.split(",")
        except Exception as e:
            print(f"[读取错误] 无法读取历史: {e}")
    return []


def load_last_inspection_seconds():
    """
    上电启动时读取上一次 inspection_history.txt 里的 inspection_time。
    只用于右下角显示，不影响历史目标读取。
    """
    if not os.path.exists(HISTORY_SAVE_FILE):
        return 0.0

    try:
        with open(HISTORY_SAVE_FILE, "r") as f:
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
        if not target_name:
            return
        if target_name not in TARGET_CN_MAP:
            return
        self.queue.append(target_name)
        if len(self.queue) > MAX_HISTORY_LEN:
            self.queue.pop(0)

    def commit(self, inspection_seconds=None):
        save_history(self.queue, inspection_seconds)

    def to_cn_string(self):
        if not self.queue:
            return ""
        # 修复：加入了 .strip() 防止隐藏的换行符破坏字典查询
        cn_list = [TARGET_CN_MAP.get(item.strip(), "未知") for item in self.queue]
        # 修复：彻底弃用容易引发字库崩溃的 ➔，改用绝对安全的 ASCII 字符 ->
        hist_str = " -> ".join(cn_list)
        return hist_str


# [新增] 导入串口库
import socket

# =========================================================
# UDP 透传设置
# =========================================================
UDP_IP = "255.255.255.255"  # 如需精准发送，请改为您电脑的实际 IP，如 "192.168.1.100"
UDP_PORT = 8080             # 上位机监听的端口

try:
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP 透传初始化成功! 将数据广播至 {UDP_IP}:{UDP_PORT}")
except Exception as e:
    print(f"UDP 初始化失败: {e}")
    udp_socket = None

try:
    from maix import uart, pinmap
    try:
        # 使用真实的 A16 和 A17 作为 UART0
        pinmap.set_pin_function("A16", "UART0_TX")
        pinmap.set_pin_function("A17", "UART0_RX")
    except Exception as e:
        print(f"引脚映射提示: {e}")

    UART_PORT = uart.UART("/dev/ttyS0", 115200)
    print("UART 串口初始化成功! (/dev/ttyS0, 映射 A16/A17)")
except Exception as e:
    print(f"UART 串口初始化失败: {e}")
    print("系统将处于调试模式，仅在终端打印 HEX 报文。")
    UART_PORT = None


DEBUG_MODE = False


class CvVisionConfig:
    # =========================================================
    # 颜色 HSV 阈值 (已恢复为亮光环境下的严格阈值，避免反光误判)
    # =========================================================
    # =========================================================
    # 颜色 HSV 阈值：极致收紧，绝对锁定真正的颜色
    # =========================================================
    # 红色收紧：防止把昏暗的黑色边沿或偏红阴影认成红色
    HSV_RED_LOWER1 = np.array([0, 70, 60])
    HSV_RED_UPPER1 = np.array([10, 255, 255])
    HSV_RED_LOWER2 = np.array([160, 70, 60])
    HSV_RED_UPPER2 = np.array([180, 255, 255])

    # 绿色微调平衡版：之前收得太紧导致连真绿圆都漏了！
    # H: 35-85 放开对偏黄绿色的宽容度
    # S: 45 允许低饱和度，但比 40 高一点点防赛道反光
    # V: 45 允许较暗的绿圆，但不低于 40 以防止纯黑赛道线被吞
    HSV_GREEN_LOWER = np.array([35, 45, 45])
    HSV_GREEN_UPPER = np.array([85, 255, 255])

    # =========================================================
    # mask 抗光线干扰参数：最小改动
    # =========================================================
    # OPEN  去除小光斑、小噪声
    # CLOSE 填补目标内部因反光/暗斑造成的小洞
    MASK_KERNEL_SIZE = 3
    MASK_CLOSE_ITER = 1

    # =========================================================
    # Y 字有效区域 ROI
    # =========================================================
    # 目的：红色范围放大后，直接屏蔽 Y 字赛道外区域，防止误识别外侧红色或反光。
    # 画面坐标：320x240。下面用比例写，换分辨率后也能基本适配。
    ENABLE_Y_ROI = True

    # Y 字分叉点高度：机位高达 19cm，视角更为俯视，分叉点在画面正中央附近
    Y_SPLIT_Y_RATIO = 0.50

    # Y 字顶部起点高度：折中取 0.26，既防远景，又不至于第一眼看不见
    Y_TOP_Y_RATIO = 0.26

    # 主干半宽：折中取 0.14，较窄，但给高速晃动留了一定容错
    Y_TRUNK_HALF_WIDTH_RATIO = 0.14

    # 左右分支终点 x 位置：向中间靠拢收缩
    Y_BRANCH_END_X_RATIO = 0.15

    # 分支半宽：折中取 0.11，保持对旁道的高度警惕
    Y_BRANCH_HALF_WIDTH_RATIO = 0.11

    # =========================================================
    # 基础轮廓过滤
    # =========================================================
    MIN_AREA = 200

    # =========================================================
    # 只有目标足够近，才判断圆/方
    # 面积门槛折中取 500：过滤大部分噪点，又能保证形状及时判定
    SHAPE_MIN_AREA = 500

    # cy 折中至 0.28：画面上方 28% 的一票否决
    SHAPE_MIN_CY_RATIO = 0.28

    # 轮廓近似比例
    # 稍微调低，保留更多顶点，防止把圆形近似成四边形
    POLY_EPSILON_RATIO = 0.015

    # 圆形在透视下允许变成椭圆
    # 填充率折中取 0.90，防 unknown 的同时保证不是乱七八糟的形状
    ELLIPSE_FILL_MIN = 0.90

    # 方形在透视下可能变成梯形
    # 19cm机位透视变弱，边长比例不会太悬殊
    SQUARE_EDGE_RATIO_MAX = 2.5

    # 方形最小矩形填充率
    # 畸变减小后，方形应该能极好地填满外接矩形
    RECT_FILL_SQUARE_MIN = 0.85

    # =========================================================
    # 防抖状态机配置
    # =========================================================
    # 高速运动下，目标在最佳识别区停留极短。窗口缩小为3，TURN_END清空缓存后需快速稳定。
    FILTER_WINDOW_SIZE = 3
    CONFIRM_VOTES = 1  # 降至1票：红色刚出现第1帧即锁定，不给下一转弯打断机会

    # =========================================================
    # 图像尺寸
    # =========================================================
    FRAME_WIDTH = 320
    FRAME_HEIGHT = 240

    # =========================================================
    # 空间过滤配置
    # =========================================================
    # 左右盲区，切掉画面的最左侧 25% 和最右侧 25%，只留中间 50% 管孔视野
    DEAD_ZONE_X_RATIO = 0.25

    # 顶部盲区，折中取 0.24，不至于一刀切把发车位的第一个目标砍没
    DEAD_ZONE_Y_TOP_RATIO = 0.24

    # 面积归一化基准 (机位降低导致目标极具放大，基准面积需要成倍增加)
    MAX_EXPECTED_AREA = 4000.0

    # 空间打分权重
    WEIGHT_AREA = 0.3
    WEIGHT_CENTER = 0.7

    # 最低有效分数 (大幅降低，允许识别边缘处的色块，防止漏判)
    MIN_VALID_SCORE = 40.0


class CvShapeDetector:
    def __init__(self, config):
        self.cfg = config
        self.roi = []
        self.history_buffer = []

    def _build_y_roi_mask(self, mask_shape):
        """
        构造 Y 字有效识别区域。
        只有 ROI 内的红/绿 mask 会被保留，ROI 外直接清零。

        默认形状：
        - 下半部分：中间竖直主干
        - 上半部分：左右两条分支
        """
        h, w = mask_shape[:2]
        roi = np.zeros((h, w), dtype=np.uint8)

        cx = w // 2
        y_split = int(h * self.cfg.Y_SPLIT_Y_RATIO)
        y_top = int(h * self.cfg.Y_TOP_Y_RATIO)

        trunk_half = int(w * self.cfg.Y_TRUNK_HALF_WIDTH_RATIO)
        branch_half = int(w * self.cfg.Y_BRANCH_HALF_WIDTH_RATIO)
        branch_thickness = max(4, branch_half * 2)

        left_x = int(w * self.cfg.Y_BRANCH_END_X_RATIO)
        right_x = int(w * (1.0 - self.cfg.Y_BRANCH_END_X_RATIO))

        # 中间主干：直接延伸到 y_top，填补远距离直线段(正前方)的空白盲区！
        cv2.rectangle(
            roi,
            (cx - trunk_half, y_top),
            (cx + trunk_half, h),
            255,
            -1
        )

        # 左右分支，用粗线表示通道宽度
        cv2.line(
            roi,
            (cx, y_split),
            (left_x, y_top),
            255,
            branch_thickness
        )
        cv2.line(
            roi,
            (cx, y_split),
            (right_x, y_top),
            255,
            branch_thickness
        )

        # 分叉点补圆，避免主干和分支连接处有缺口
        cv2.circle(
            roi,
            (cx, y_split),
            max(trunk_half, branch_half),
            255,
            -1
        )

        return roi

    def _get_color_mask(self, hsv_img, color_type):
        """
        根据颜色生成 mask

        最小改动版：
        1. 红/绿 HSV 阈值在 CvVisionConfig 中放宽到暗光环境；
        2. 对 mask 做开运算，去掉小光斑、小噪声；
        3. 对 mask 做闭运算，补目标内部小洞；
        4. 在 mask 阶段提前屏蔽左右/顶部盲区；
        5. 启用 Y 字有效区域 ROI，ROI 外直接清零，防止识别 Y 字外目标。
        """
        if color_type == "red":
            mask1 = cv2.inRange(
                hsv_img,
                self.cfg.HSV_RED_LOWER1,
                self.cfg.HSV_RED_UPPER1
            )
            mask2 = cv2.inRange(
                hsv_img,
                self.cfg.HSV_RED_LOWER2,
                self.cfg.HSV_RED_UPPER2
            )
            mask = cv2.bitwise_or(mask1, mask2)

        elif color_type == "green":
            mask = cv2.inRange(
                hsv_img,
                self.cfg.HSV_GREEN_LOWER,
                self.cfg.HSV_GREEN_UPPER
            )

        else:
            mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)

        # =====================================================
        # 提前屏蔽无效区域
        # 目的：减少左右边缘、顶部远处、旁边车道和环境光干扰
        # =====================================================
        h, w = mask.shape[:2]
        dead_x = int(w * self.cfg.DEAD_ZONE_X_RATIO)
        dead_y = int(h * self.cfg.DEAD_ZONE_Y_TOP_RATIO)

        mask[:, :dead_x] = 0
        mask[:, w - dead_x:] = 0
        mask[:dead_y, :] = 0

        # =====================================================
        # Y 字有效区域过滤
        # 红色范围放大后，这一步非常重要：Y 字外的颜色目标直接不参与识别。
        # =====================================================
        if self.cfg.ENABLE_Y_ROI:
            y_roi = self._build_y_roi_mask(mask.shape)
            mask = cv2.bitwise_and(mask, y_roi)

        # =====================================================
        # 形态学滤波
        # open：先腐蚀后膨胀，去除小噪声、小光斑
        # close：先膨胀后腐蚀，填补目标内部小洞
        # =====================================================
        k = self.cfg.MASK_KERNEL_SIZE
        kernel = np.ones((k, k), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=self.cfg.MASK_CLOSE_ITER
        )

        return mask

    def _detect_shape(self, hull, area, perimeter, x, y, w, h):
        """
        只识别 circle / square 两种形状。

        核心原则：
        1. 目标远时不判断形状，返回 unknown
        2. 方形：近处 + 凸四边形 + 边长比例合理
        3. 圆形：近处 + 椭圆拟合填充率高
        """
        cx = x + w // 2
        cy = y + h // 2

        circularity = 0.0
        if perimeter > 0:
            circularity = (4 * np.pi * area) / (perimeter ** 2)

        # 默认值，用于调试显示
        shape = "unknown"
        vertex_count = 0
        ellipse_fill = 0.0
        rect_fill = 0.0
        edge_ratio = 999.0
        # =====================================================
        # 远处不判断形状
        # =====================================================
        if area < self.cfg.SHAPE_MIN_AREA:
            return shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio

        if cy < self.cfg.FRAME_HEIGHT * self.cfg.SHAPE_MIN_CY_RATIO:
            return shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio

        # =====================================================
        # 多维度特征提取 (应对透视变换与畸变)
        # =====================================================
        # 1. 轮廓多边形近似 (主要提取顶点数)
        epsilon = self.cfg.POLY_EPSILON_RATIO * perimeter
        approx = cv2.approxPolyDP(hull, epsilon, True)
        vertex_count = len(approx)

        # 2. 最小外接矩形特征
        rect = cv2.minAreaRect(hull)
        rw, rh = rect[1]
        if rw > 0 and rh > 0:
            rect_area = rw * rh
            rect_fill = area / rect_area
            # 透视下的矩形长宽比，防止极其细长的噪点
            rect_aspect_ratio = max(rw, rh) / min(rw, rh)
        else:
            rect_area = 0
            rect_fill = 0
            rect_aspect_ratio = 999

        # 3. 椭圆拟合特征
        ellipse_fill = 0
        if len(hull) >= 5:
            try:
                ellipse = cv2.fitEllipse(hull)
                (_, _), (axis1, axis2), _ = ellipse
                if axis1 > 0 and axis2 > 0:
                    ellipse_area = np.pi * (axis1 / 2.0) * (axis2 / 2.0)
                    ellipse_fill = area / ellipse_area
            except Exception:
                pass

        # 4. 圆形度 (Circularity)
        circularity = (4 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0

        # =====================================================
        # 综合判定逻辑
        # =====================================================
        # 【方形判定】
        # 透视下的正方形可能产生 4~5 个顶点，外接矩形填充率依然较高
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
            
            # 条件：边长比例不离谱，外接矩形填充率达标，并且长宽比不过分瘦长
            if (edge_ratio <= self.cfg.SQUARE_EDGE_RATIO_MAX 
                and rect_fill >= self.cfg.RECT_FILL_SQUARE_MIN 
                and rect_aspect_ratio <= 3.0):
                # 排除被错误识别为 4-5 边形的完美圆形
                if ellipse_fill < 0.93 or circularity < 0.8:
                    is_square = True

        if is_square:
            shape = "square"
        else:
            # 【圆形判定】
            # 透视下的圆形变成椭圆，顶点数较多 (>=6)，椭圆拟合填充率非常高
            if vertex_count >= 5 and len(hull) >= 5:
                # 满足以下任意一条强特征组合即可认定为圆：
                # 1. 经典圆：顶点较多且椭圆填充率达标
                condition1 = (vertex_count >= 6 and ellipse_fill >= self.cfg.ELLIPSE_FILL_MIN)
                # 2. 强透视椭圆：即便形状因为畸变被拟合成 5 个顶点，但它的确是个极其贴合的椭圆
                condition2 = (vertex_count == 5 and ellipse_fill >= 0.90)
                
                if condition1 or condition2:
                    shape = "circle"

        # 强制消除 unknown：满足"绝对不允许出现色块输出"的强制要求
        if shape == "unknown":
            # 【加大圆方区分力度】
            # 1. 顶点数极少（<=4），毫无疑问是方块（透视形变）
            if vertex_count <= 4:
                shape = "square"
            # 2. 顶点数较多（>=6），且椭圆填充率极高或圆度极高，毫无疑问是圆
            elif vertex_count >= 6 and (ellipse_fill > 0.85 or circularity > 0.82):
                shape = "circle"
            else:
                # 3. 处于中间模糊地带（如顶点数5）的终极对决
                # 高机位下，方块极易填满外接矩形(>0.9)，圆的矩形填充率理论上限 pi/4 ≈ 0.785
                if rect_fill > 0.88:
                    shape = "square"
                # 如果椭圆填充率极高，且显著大于矩形填充率
                elif ellipse_fill > 0.90 and ellipse_fill > (rect_fill + 0.08):
                    shape = "circle"
                else:
                    shape = "square"

        return shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio

    def _select_best_target(self, current_frame_results):
        """
        基于空间盲区与启发式加权评分选择最可信目标。
        unknown 不参与最终选择。
        """
        best_target_name = "none"
        highest_score = -1.0

        center_x = self.cfg.FRAME_WIDTH / 2.0
        dead_zone_margin_x = self.cfg.FRAME_WIDTH * self.cfg.DEAD_ZONE_X_RATIO
        dead_zone_margin_y_top = self.cfg.FRAME_HEIGHT * self.cfg.DEAD_ZONE_Y_TOP_RATIO

        for res in current_frame_results:
            if res.get("shape") == "discarded":
                continue

            cx = res["cx"]
            cy = res["cy"]
            area = res["area"]
            color = res["color"]
            shape = res["shape"]

            # 【关键修改】：彻底解除形状束缚！
            # 远处的目标受透视和像素限制，必定是 unknown。
            # 因为我们在前一步已经用严苛的 HSV 阈值过滤了杂质，只要色块出现了，就一定是真正的目标。
            # 直接允许 unknown 参与竞标，抢在第一时间把颜色锁定并发送给单片机！
            # 既然能进入 current_frame_results，说明已经通过了红框和黄框的双重过滤！
            # 它们全都是合法区域内的高价值候选者。

            # 面积分数
            score_area = min(
                100.0,
                (area / self.cfg.MAX_EXPECTED_AREA) * 100.0
            )

            # 居中分数
            dist_to_center = abs(cx - center_x)
            score_center = max(
                0.0,
                100.0 - (dist_to_center / center_x) * 100.0
            )

            # 总分
            total_score = (
                score_area * self.cfg.WEIGHT_AREA +
                score_center * self.cfg.WEIGHT_CENTER
            )

            if DEBUG_MODE:
                print(
                    f"[{color}_{shape}] "
                    f"area={area:.1f} "
                    f"score_area={score_area:.1f} "
                    f"score_center={score_center:.1f} "
                    f"total={total_score:.1f}"
                )

            if total_score > highest_score and total_score >= self.cfg.MIN_VALID_SCORE:
                highest_score = total_score
                best_target_name = f"{color}_{shape}"

        return best_target_name

    def process_frame(self, img_maix, history_len=0):
        """
        处理一帧图像
        """


        img_cv = image.image2cv(img_maix, copy=False)
        hsv_img = cv2.cvtColor(img_cv, cv2.COLOR_RGB2HSV)

        current_frame_results = []

        for color_name in ["red", "green"]:
            mask = self._get_color_mask(hsv_img, color_name)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                raw_area = cv2.contourArea(cnt)
                if raw_area < self.cfg.MIN_AREA:
                    continue

                # 凸包修复反光缺口
                hull = cv2.convexHull(cnt)

                area = cv2.contourArea(hull)
                perimeter = cv2.arcLength(hull, True)

                if area < self.cfg.MIN_AREA:
                    continue

                if perimeter <= 0:
                    continue

                x, y, w, h = cv2.boundingRect(hull)
                cx = x + w // 2
                cy = y + h // 2

                # =====================================================
                # 物理死区与黄框 ROI 终极过滤（前置）
                # =====================================================
                center_x = self.cfg.FRAME_WIDTH / 2.0
                dead_zone_margin_x = self.cfg.FRAME_WIDTH * self.cfg.DEAD_ZONE_X_RATIO
                dead_zone_margin_y_top = self.cfg.FRAME_HEIGHT * self.cfg.DEAD_ZONE_Y_TOP_RATIO

                discard_reason = ""
                # 左右边界与顶部红线盲区
                if cx < dead_zone_margin_x or cx > self.cfg.FRAME_WIDTH - dead_zone_margin_x:
                    discard_reason = "X边缘"
                elif cy < dead_zone_margin_y_top:
                    discard_reason = "Y顶部"
                # Removed redundant and buggy manual trunk/branch filtering,
                # because `_build_y_roi_mask` already accurately masks the Y-ROI at the pixel level.

                if discard_reason:
                    current_frame_results.append({
                        "color": color_name,
                        "shape": "discarded",
                        "x": x, "y": y, "w": w, "h": h, "cx": cx, "cy": cy,
                        "area": area,
                        "reason": discard_reason
                    })
                    continue

                shape, vertex_count, circularity, ellipse_fill, rect_fill, edge_ratio = self._detect_shape(
                    hull,
                    area,
                    perimeter,
                    x,
                    y,
                    w,
                    h
                )

                current_frame_results.append({
                    "color": color_name,
                    "shape": shape,
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                    "area": area,
                    "roundness": circularity,
                    "vertex": vertex_count,
                    "ellipse_fill": ellipse_fill,
                    "rect_fill": rect_fill,
                    "edge_ratio": edge_ratio
                })

        target_name = self._select_best_target(current_frame_results)

        # =====================================================
        # 防抖确认
        # =====================================================
        self.history_buffer.append(target_name)

        if len(self.history_buffer) > self.cfg.FILTER_WINDOW_SIZE:
            self.history_buffer.pop(0)

        stable_result = None

        # 过滤出所有有效的候选目标，不让 "none" 参与竞选投票
        valid_targets = [t for t in self.history_buffer if t != "none"]
        if valid_targets:
            best_target = max(set(valid_targets), key=valid_targets.count)
            if valid_targets.count(best_target) >= self.cfg.CONFIRM_VOTES:
                stable_result = best_target
                
        # 如果缓存满了都没能决出胜负，就输出 "none" 清空状态
        if stable_result is None and len(self.history_buffer) == self.cfg.FILTER_WINDOW_SIZE:
            stable_result = "none"

        return current_frame_results, stable_result

    def draw_debug_info(self, img_maix, results, stable_result):
        """
        屏幕调试显示
        """
        # 左右盲区线
        dead_zone_margin_x = int(
            self.cfg.FRAME_WIDTH * self.cfg.DEAD_ZONE_X_RATIO
        )

        img_maix.draw_line(
            dead_zone_margin_x,
            0,
            dead_zone_margin_x,
            self.cfg.FRAME_HEIGHT,
            COLOR_R,
            1
        )

        img_maix.draw_line(
            self.cfg.FRAME_WIDTH - dead_zone_margin_x,
            0,
            self.cfg.FRAME_WIDTH - dead_zone_margin_x,
            self.cfg.FRAME_HEIGHT,
            COLOR_R,
            1
        )

        # 顶部盲区线
        dead_zone_margin_y_top = int(
            self.cfg.FRAME_HEIGHT * self.cfg.DEAD_ZONE_Y_TOP_RATIO
        )

        img_maix.draw_line(
            0,
            dead_zone_margin_y_top,
            self.cfg.FRAME_WIDTH,
            dead_zone_margin_y_top,
            COLOR_R,
            1
        )

        # 近距离形状识别线
        shape_min_y = int(
            self.cfg.FRAME_HEIGHT * self.cfg.SHAPE_MIN_CY_RATIO
        )

        img_maix.draw_line(
            0,
            shape_min_y,
            self.cfg.FRAME_WIDTH,
            shape_min_y,
            COLOR_Y,
            1
        )

        # Y 字 ROI 边界提示线，方便现场调参数
        if self.cfg.ENABLE_Y_ROI:
            cx = self.cfg.FRAME_WIDTH // 2
            y_top = int(self.cfg.FRAME_HEIGHT * self.cfg.Y_TOP_Y_RATIO)
            trunk_half = int(self.cfg.FRAME_WIDTH * self.cfg.Y_TRUNK_HALF_WIDTH_RATIO)
            
            # 画出主干（也是当前最主要的 ROI 识别区）的黄色边界框
            # 这样用户就能一眼看出左右两边和顶部被砍掉的部分
            img_maix.draw_rect(
                cx - trunk_half, 
                y_top, 
                trunk_half * 2, 
                self.cfg.FRAME_HEIGHT - y_top, 
                COLOR_Y, 
                2
            )
            
            # 顶部 20% 死区屏蔽线（贯穿全屏，起到警示作用）
            img_maix.draw_line(
                0,
                y_top,
                self.cfg.FRAME_WIDTH,
                y_top,
                COLOR_Y,
                2
            )

        for res in results:
            if res.get("shape") == "discarded":
                # 被屏蔽的色块，用白色细线画出并标注原因
                img_maix.draw_rect(res["x"], res["y"], res["w"], res["h"], COLOR_W, 1)
                img_maix.draw_string(res["x"], max(0, res["y"] - 15), f"屏蔽:{res['reason']}", COLOR_W)
                continue

            if res["color"] == "red":
                rect_color = COLOR_R
            else:
                rect_color = COLOR_G

            img_maix.draw_rect(
                res["x"],
                res["y"],
                res["w"],
                res["h"],
                rect_color,
                2
            )

            img_maix.draw_cross(
                res["cx"],
                res["cy"],
                COLOR_W,
                5
            )

            # 调试信息：
            # V = 顶点数
            # C = 圆度
            # E = 椭圆填充率
            # R = 矩形填充率
            label = (
                f"{res['shape']} "
                f"V{res['vertex']} "
                f"C{res['roundness']:.2f} "
                f"E{res['ellipse_fill']:.2f} "
                f"R{res['rect_fill']:.2f}"
            )

            img_maix.draw_string(
                res["x"],
                max(0, res["y"] - 20),
                label,
                rect_color
            )

        if stable_result:
            img_maix.draw_string(
                2,
                210,
                f"STABLE: {stable_result}",
                color=COLOR_W,
                scale=1.5
            )



# =========================================================
# UART 通信协议
# =========================================================
def send_uart_command(uart_port, target_name):
    """
    通信协议:
    AA 55 [转向信号] [颜色信号] [形状信号] [校验和]

    颜色:
    green = 0x00
    red   = 0x01

    转向 (红右绿左):
    green = 左转 = 0x00
    red   = 右转 = 0x01

    形状:
    square = 0x00
    circle = 0x01

    checksum = (turn + color + shape) & 0xFF
    """
    if target_name is None:
        return

    if "_" not in target_name:
        return

    color_str, shape_str = target_name.split("_")

    if color_str not in ["red", "green"]:
        return

    # 注释掉对 shape_str 的硬性拦截，允许远处的 unknown 也发送（为了抢时间先传颜色定方向）
    # if shape_str not in ["circle", "square"]:
    #     return

    # 颜色
    val_color = 0x01 if color_str == "red" else 0x00

    # 转向：红右绿左
    val_turn = 0x01 if color_str == "red" else 0x00

    # 形状
    if shape_str == "circle":
        val_shape = 0x01
    elif shape_str == "square":
        val_shape = 0x00
    else:
        val_shape = 0xFF  # unknown

    # 校验和
    checksum = (val_turn + val_color + val_shape) & 0xFF

    packet = bytearray([
        0xAA,
        0x55,
        val_turn,
        val_color,
        val_shape,
        checksum
    ])

    if uart_port:
        try:
            uart_port.write(bytes(packet))
        except Exception as e:
            print(f"UART 发送失败: {e}")
            return False

    # 简化：只在串口实际发送时返回成功标志，日志交给上层统一精简打印
    return True


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
        self.uart_send_locked = False  # [防残留核心] 串口发送锁，防止转弯途中重发旧目标
        # 上电先读取上一次保存的巡检时间，用于右下角显示
        self.inspect_display_seconds = load_last_inspection_seconds()

        # [方案 A] 颜色分离发送策略
        self.last_sent_target = None  # 追踪上次发送的完整目标，避免重复
        self.last_sent_color = None  # 追踪上次识别的颜色
        self.color_stable_frames = 0  # 颜色稳定帧计数，2帧后发送

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
                # 修复：改用最稳定的白色渲染
                img_maix.draw_string(10, 90, hist_str, COLOR_W)

            # 修复：弃用容易触发纯黑 Bug 的 -1 填充，改为安全的白色外框
            img_maix.draw_rect(self.btn_x, self.btn_y, self.btn_w, self.btn_h, COLOR_W, 2)
            img_maix.draw_string(self.btn_x + 20, self.btn_y + 10, "点击开始发车", COLOR_W)

            # 上电未点击开始前，也在右下角显示上一次 txt 中的 inspection_time
            self._draw_inspection_time(img_maix)

            self.disp.show(img_maix)

            try:
                x, y, pressed = ts.read()
                # KISS 交互：只要侦测到屏幕被按压，立刻全屏触发发车！
                if pressed:
                    review_mode = False
                    self.history.clear()
                    self.pending_target = None
                    self.history_committed = False
                    self.inspect_start_time = _std_time.time()
                    self.inspect_end_time = None
                    self.inspect_display_seconds = 0.0
                    print("[巡检计时] 点击开始发车，计时清零并开始")
            except Exception:
                pass

        del ts

    def _configure_camera(self):
        # =========================================================
        # 摄像头参数
        # =========================================================
        # 暗光环境适当提高曝光和增益
        # 曝光不要太高，否则小车运动时会拖影，方形角点会变圆
        self.cam.exp_mode(camera.AeMode.Manual)
        self.cam.exposure(20000)
        self.cam.gain(20)

    def _commit_history_with_time(self):
        if self.history_committed:
            return

        if self.inspect_start_time is None:
            self.inspect_start_time = _std_time.time()

        # 结束时间 = 写入 txt 文件前一瞬间 
        self.inspect_end_time = _std_time.time()
        inspection_seconds = self.inspect_end_time - self.inspect_start_time
        self.inspect_display_seconds = inspection_seconds

        self.history.commit(inspection_seconds)
        self.history_committed = True
        print(f"[巡检计时] 巡检用时: {inspection_seconds:.2f}s，已锁定")

        # 将死亡回放通过串口发回给单片机/蓝牙终端
        if UART_PORT:
            try:
                targets_cn = self.history.to_cn_string()
                replay_msg = f"Path: {targets_cn}, Time: {inspection_seconds:.2f}s\r\n"
                UART_PORT.write(replay_msg.encode("utf-8"))
            except Exception as e:
                print(f"[UART 异常] 回放回传失败: {e}")

    def _get_inspection_elapsed_seconds(self):
        """
        右下角显示用：
        1. 上电未点击开始：显示上一次 txt 里的 inspection_time；
        2. 点击开始后：从 0 开始计时；
        3. 录入满 4 个后：锁定最终时间。
        """
        if self.inspect_start_time is None:
            return self.inspect_display_seconds

        if self.history_committed:
            return self.inspect_display_seconds

        return _std_time.time() - self.inspect_start_time

    def _draw_inspection_time(self, img_maix):
        """
        右下角输出巡检时间字符。
        """
        elapsed = self._get_inspection_elapsed_seconds()
        img_maix.draw_string(210, 218, f"T:{elapsed:.2f}s", COLOR_W)

    def _process_uart_input(self):
        # 初始化行缓存和打印限流时间戳
        if not hasattr(self, 'uart_rx_buffer'):
            self.uart_rx_buffer = bytearray()
            self.last_uart_print_time = 0.0

        if UART_PORT:
            try:
                in_data = UART_PORT.read()
                if in_data:
                    self.uart_rx_buffer.extend(in_data)
                    
                    # 按照换行符进行"组包"处理，防止频繁系统调用卡死主循环
                    while b'\n' in self.uart_rx_buffer:
                        idx = self.uart_rx_buffer.index(b'\n')
                        line_data = self.uart_rx_buffer[:idx+1]
                        self.uart_rx_buffer = self.uart_rx_buffer[idx+1:]
                        
                        # 1. 解码原始数据
                        try:
                            text = line_data.decode("utf-8", "ignore")
                            clean_text = text.strip()
                            
                            # 2. 拼接视觉状态与帧率 (VIS:red_circle, FPS:30.1)
                            vis_target = self.current_target if getattr(self, 'current_target', None) else "none"
                            cur_fps = time.fps()
                            if clean_text:
                                telemetry_str = f"{clean_text},VIS:{vis_target},FPS:{cur_fps:.1f}\n"
                            else:
                                telemetry_str = f"VIS:{vis_target},FPS:{cur_fps:.1f}\n"

                            # 3. 发送附加了视觉信息的 UDP 报文到上位机
                            if udp_socket:
                                try:
                                    udp_socket.sendto(telemetry_str.encode("utf-8"), (UDP_IP, UDP_PORT))
                                except Exception:
                                    pass
                            
                            # 4. 简化底层遥测输出，只在关键时刻或低频心跳时输出
                            current_time = _std_time.time()
                            if clean_text and (current_time - self.last_uart_print_time > 1.0):
                                # print(f"[{cur_fps:.1f}fps] Odo:{clean_text.split('Odo:')[1].split(',')[0]} Vis:{vis_target}")
                                self.last_uart_print_time = current_time

                            if text.startswith("TURN_DECISION:"):
                                dec_str = text.strip()
                                now = _std_time.time()
                                sent_time = getattr(self, "last_sent_time", 0)
                                
                                if "VisRead=-1" in dec_str:
                                    self.last_turn_was_fallback = True
                                else:
                                    self.last_turn_was_fallback = False

                                if sent_time > 0:
                                    dt = now - sent_time
                                    print(f"\n>>>> [时序诊断] 单片机上报切弯决策: {dec_str} <<<<")
                                    print(f"     -> 视觉上一次发送颜色距今: {dt:.3f} 秒\n")
                                else:
                                    print(f"\n>>>> [时序诊断] 单片机上报切弯决策: {dec_str} <<<<")
                                    print(f"     -> 视觉尚未发送过任何有效颜色！\n")
                                continue

                            self.uart_rx_text += text
                            
                            if "TURN_START\n" in self.uart_rx_text:
                                self.is_turning = True
                                self._uart_turn_event = True # Y 节点：既要闭眼，又要记录前一个稳定目标！
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_START\n", "")
                                # 核心排查点：打印单片机要结果时的视觉状态
                                sent_color = getattr(self, "last_sent_target", "无(未锁定)")
                                # print(f"==== [单片机已切弯] (Y字) 单片机已读取数据并开始转向！ 视觉此时已发送给它的颜色是: {sent_color} ====")

                            elif "TURN_BLIND_ONLY\n" in self.uart_rx_text:
                                self.is_turning = True # L 节点：只要闭眼，绝对不记录历史！
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_BLIND_ONLY\n", "")
                                print("[视觉防抖] 收到 TURN_BLIND_ONLY (L字拐角)，仅进入过弯盲区保护，不记录目标！")

                            elif "TURN_END\n" in self.uart_rx_text:
                                self.uart_rx_text = self.uart_rx_text.replace("TURN_END\n", "")
                                self.is_turning = False
                                self.uart_send_locked = False
                                # 【核心修复】：闭眼期间虽然没处理数据，但图像识别底层还在偷偷把看到的残影塞进防抖队列。
                                # 睁眼的一瞬间，必须把这这段时间的"潜意识记忆"全部清空！否则一睁眼就会立刻触发锁定残影！
                                self.detector.history_buffer.clear()
                                self.current_target = None
                                self.pending_target = None
                                self.display_cn_target = "无"
                                # [方案A] 转弯恢复后，重置颜色追踪状态，确保新目标能快速发送
                                self.last_sent_color = None
                                self.last_sent_target = None
                                self.color_stable_frames = 0
                                
                                print("[视觉恢复] 收到 TURN_END，陀螺仪已稳定，缓存已清空，恢复纯净视野识别！")
                            if "INSPECT_START\n" in self.uart_rx_text:
                                print(f"[视觉复位] 收到单片机发车信号 INSPECT_START，视觉状态已重置，开始新一轮巡检！")
                                self.history_committed = False
                                self.history.clear()
                                self.inspect_start_time = _std_time.time()
                                self.ui_state = "running"
                                self.uart_rx_text = self.uart_rx_text.replace("INSPECT_START\n", "")

                            if "INSPECT_END" in self.uart_rx_text and not self.history_committed:
                                print(f"[巡检完成] 收到单片机 INSPECT_END 信号，结束巡检")
                                self._commit_history_with_time()
                                self.uart_rx_text = self.uart_rx_text.replace("INSPECT_END", "")

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
            print("[历史记录] 检测到转向，但没有可录入的前一稳定目标")
            return
        # 严格过滤：只允许明确的 circle/square 录入，unknown 绝不录入
        if "unknown" in self.pending_target:
            print(f"[历史记录] 拒绝录入模糊目标: {self.pending_target}，等待形状确认")
            return

        self.history.append_target(self.pending_target)
        print(f"[历史记录] 角度突变确认，第 {len(self.history.queue)} 次录入: {self.pending_target}")
        
        # 物理转向发生！此时不仅单片机亮灯，屏幕上也显示"已入弯！"
        confirmed_str = TARGET_CN_MAP.get(self.pending_target, "无")
        self.display_cn_target = f"转弯点亮: {confirmed_str}"
        self.pending_target = None

    def _handle_stable_result(self, stable_result, turn_event):
        if turn_event:
            # 【核心策略】：只有陀螺仪稳定时（即收到TURN_START瞬间，车身还未真正旋转甩尾）的图形才能写入！
            # 拒绝在转弯过程中去识别任何东西，防止把背景噪点误认为目标。
            if getattr(self, "last_turn_was_fallback", False):
                print("[防抖保护] 单片机因未及时收到视觉数据使用了盲猜回退(VisRead=-1)，拒绝录入当前目标，防止时序错乱！")
            elif self.current_target and self.current_target != "none" and "unknown" not in self.current_target:
                self.pending_target = self.current_target
                self._record_pending_target_by_turn()
            else:
                target_name = self.current_target if self.current_target else "无"
                print(f"[防抖保护] 转向瞬间目标不明确({target_name})，拒绝录入噪点！")

            self.is_turning = True  # 立刻闭眼
            self.current_target = None
            self.display_cn_target = "转弯盲区"
            self.uart_send_locked = True
            self.last_sent_color = None  # 重置颜色记录，以便下个直道重新发送
            self.color_stable_frames = 0
            self.detector.history_buffer.clear()
            return

        # 陀螺仪转弯期间，忽略一切视觉结果（这通常在看清形状后触发，或者强制盲转 L 角触发）
        if getattr(self, "is_turning", False):
            return

        # ============ [方案 A] 初始接近阶段：快速发送纯颜色 ============
        if stable_result and stable_result != "none":
            if not self.history_committed:
                # 从目标名提取颜色（如 "red_circle" -> "red"）
                color = stable_result.split("_")[0]

                # 颜色改变或首次识别时重置计数
                if color != self.last_sent_color:
                    self.last_sent_color = color
                    self.color_stable_frames = 0
                    self.last_sent_target = None  # 强制重新发送

                # 计数递增
                self.color_stable_frames += 1

                # 颜色稳定 1 帧后，发送纯颜色目标（强制 unknown，避免初期形状误判）
                pure_color_target = f"{color}_unknown"
                if (self.color_stable_frames >= 1 and
                    self.last_sent_target != pure_color_target and
                    not self.uart_send_locked):
                    # 只发送纯颜色，不发形状，避免初期误识别形状误导单片机
                    send_uart_command(UART_PORT, pure_color_target)
                    self.last_sent_target = pure_color_target
                    self.last_sent_time = _std_time.time()
                    print(f"\n>>>> [视觉已锁定颜色] 耗时 {self.color_stable_frames} 帧，成功识别到: {color}色，已发送报文给单片机！ <<<<\n")

                # 更新待转目标
                self.current_target = stable_result
                self.pending_target = stable_result
                target_str = TARGET_CN_MAP.get(self.current_target, "无")
                self.display_cn_target = f"已锁定待转: {target_str}"

        # 当视野里确认没有目标了
        if stable_result == "none" or stable_result is None:
            self.current_target = None
            # [核心修复] 不要在这里重置 last_sent_target，否则一旦边缘抖动就会导致疯狂重复发送串口数据！
            # 它们会在 TURN_END 时被统一安全重置。
            self.color_stable_frames = 0
            if not self.pending_target:
                self.display_cn_target = "无"
            if not turn_event:
                self.uart_send_locked = False

    def run(self):
        self._load_font()
        self._review_history_until_start()
        self._configure_camera()

        print("视觉系统初始化完成")
        print("策略: 远处只看颜色和位置，近处再判断圆/方")
        print("记录策略: 先缓存稳定最高分目标，检测到 MCU 回传 TURN 信号时录入")
        print("UART 协议: AA 55 turn color shape checksum")
        print("暗光优化: 绿色 HSV 阈值已降低 S/V 下限，并启用 mask 闭运算滤波")

        while not app.need_exit():
            turn_event = self._uart_turn_event
            self._uart_turn_event = False

            img_maix = self.cam.read()
            if img_maix is None:
                continue
            self._process_uart_input()

            history_len = len(self.history.queue)
            results, stable_result = self.detector.process_frame(img_maix, history_len)
            self.detector.draw_debug_info(img_maix, results, stable_result)
            img_maix.draw_string(2, 2, f"FPS: {time.fps():.1f}", color=COLOR_W)

            self._handle_stable_result(stable_result, turn_event)

            img_maix.draw_rect(80, 0, 240, 25, image.Color.from_rgb(0, 0, 0), -1)
            img_maix.draw_string(85, 2, f"实时参数: {self.display_cn_target}", COLOR_W)
            self._draw_inspection_time(img_maix)
            self.disp.show(img_maix)


def main():
    inspection_app = InspectionCarApp()
    inspection_app.run()


if __name__ == "__main__":
    main()
