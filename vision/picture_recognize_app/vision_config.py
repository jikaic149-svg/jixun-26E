"""MaixCAM 巡检识别、串口和掉电保存配置。"""

MODEL_PATH = "/root/mymodel/jixun_pictureRecognize_yolo11n_v2.mud"

CONF_TH = 0.60
IOU_TH = 0.45
DUAL_BUFF = False
CENTER_DEAD_ZONE = 10

# -1 表示处理全部类别；0..5 对应下方 CLASS_NAMES。
TARGET_CLASS_ID = -1

# 0=自动判断方形/圆形，1=只按方形处理，2=只按圆形处理。
TARGET_SHAPE_ID = 0

DARK_BORDER_THRESHOLDS = [(0, 60, -128, 127, -128, 127)]
BORDER_PRIMARY_SCALE = 1.20
BORDER_FALLBACK_SCALE = 1.70
BORDER_MIN_SCORE = 0.55
RECT_THRESHOLD = 2000

DEBUG_PRINT = False
BINARY_DEBUG_VIEW = False
SHOW_DEBUG_OVERLAYS = False
SHOW_STATUS = True
PROCESS_ALL_GEOMETRY_TARGETS = False

# MCU 使用 UART0 PB0(TX)/PB1(RX)；MaixCAM 使用 UART0 A16(TX)/A17(RX)。
UART_DEVICE = "/dev/ttyS0"
UART_BAUD_RATE = 115200
UART_TX_PIN = "A16"
UART_RX_PIN = "A17"
UART_REOPEN_MS = 2000
UART_RX_LIMIT = 256

HISTORY_FILE = "/root/inspection_history.txt"
HISTORY_TEMP_FILE = "/root/inspection_history.tmp"
MAX_RECORDS = 14
DETECTION_STABLE_FRAMES = 3
DETECTION_CENTER_TOLERANCE_PX = 35

# 顺序必须和 data.yaml 及 .mud 完全一致。
CLASS_NAMES = ("dog", "cat", "pig", "sedan", "bus", "truck")
CLASS_NAMES_CN = ("狗", "猫", "猪", "轿车", "大巴", "卡车")
CLASS_COLORS_RGB = (
    (40, 120, 255),
    (0, 220, 220),
    (255, 220, 0),
    (40, 220, 80),
    (190, 80, 255),
    (255, 80, 140),
)
