"""MaixCAM 图片识别、几何中心与调试显示配置。"""

# 模型文件必须和对应 .cvimodel 一起放到 /root/mymodel。
MODEL_PATH = "/root/mymodel/jixun_pictureRecognize_yolo11n_v2.mud"

CONF_TH = 0.60
IOU_TH = 0.45
DUAL_BUFF = False
CENTER_DEAD_ZONE = 10

# 类别 -1=全部，0..5=dog/cat/pig/sedan/bus/truck。
TARGET_CLASS_ID = -1

# 形状 0=通用外框中心，1=矩形，2=圆/椭圆。
TARGET_SHAPE_ID = 0

# 深色外框检测参数。
DARK_BORDER_THRESHOLDS = [(0, 60, -128, 127, -128, 127)]
BORDER_PRIMARY_SCALE = 1.20
BORDER_FALLBACK_SCALE = 1.70
BORDER_MIN_SCORE = 0.55
RECT_THRESHOLD = 2000

# 运行与调试开关。
DEBUG_PRINT = False
BINARY_DEBUG_VIEW = False
SHOW_DEBUG_OVERLAYS = False
SHOW_STATUS = True
PROCESS_ALL_GEOMETRY_TARGETS = False

# 类别顺序必须与 data.yaml 和 .mud 完全一致。
CLASS_NAMES = ("dog", "cat", "pig", "sedan", "bus", "truck")
CLASS_COLORS_RGB = (
    (40, 120, 255),
    (0, 220, 220),
    (255, 220, 0),
    (40, 220, 80),
    (190, 80, 255),
    (255, 80, 140),
)
