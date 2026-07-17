"""MaixCAM 图片类别识别与标识物几何中心定位入口。"""

from maix import app, camera, display, image, nn, time

import vision_config as config
from geometry_center import find_geometry_center


COLORS = [image.Color.from_rgb(*rgb) for rgb in config.CLASS_COLORS_RGB]
WHITE = image.Color.from_rgb(255, 255, 255)
GREEN = image.Color.from_rgb(40, 255, 80)
RED = image.Color.from_rgb(255, 40, 40)
YELLOW = image.Color.from_rgb(255, 220, 0)
CYAN = image.Color.from_rgb(0, 220, 255)


def select_objects(objs):
    """选择需要运行传统视觉的控制目标；其余目标仍显示类别。"""
    if config.TARGET_CLASS_ID < 0:
        candidates = list(objs)
    else:
        candidates = [
            obj for obj in objs if obj.class_id == config.TARGET_CLASS_ID
        ]
    if not candidates:
        return []
    if config.PROCESS_ALL_GEOMETRY_TARGETS:
        return candidates
    return [max(candidates, key=lambda obj: obj.score)]


def make_display_frame(img):
    """按开关返回彩色原图，或返回不影响识别输入的二值显示副本。"""
    if not config.BINARY_DEBUG_VIEW:
        return img
    try:
        preview = img.copy()
        converted = preview.binary(config.DARK_BORDER_THRESHOLDS)
        return converted if converted is not None else preview
    except Exception as error:
        if config.DEBUG_PRINT:
            print("binary debug view failed:", error)
        return img


def draw_categories(img, detector, objs):
    """所有 YOLO 目标只绘制类别名称，不绘制检测框。"""
    for obj in objs:
        color = COLORS[obj.class_id % len(COLORS)]
        img.draw_string(
            obj.x,
            max(0, obj.y - 14),
            detector.labels[obj.class_id],
            color=color,
        )


def draw_result(img, detector, obj, geometry, roi, frame_cx, frame_cy):
    """默认只绘制红色几何中心；详细框线由调试开关控制。"""
    color = COLORS[obj.class_id % len(COLORS)]
    label = detector.labels[obj.class_id]

    if config.SHOW_DEBUG_OVERLAYS:
        box_cx = obj.x + obj.w // 2
        box_cy = obj.y + obj.h // 2
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color, thickness=2)
        img.draw_cross(box_cx, box_cy, color=WHITE, size=5, thickness=1)
        img.draw_rect(
            roi[0], roi[1], roi[2], roi[3], color=WHITE, thickness=1
        )

    if geometry is None:
        if config.SHOW_DEBUG_OVERLAYS:
            img.draw_string(
                obj.x,
                max(0, obj.y - 30),
                "%s %.2f GEO:NONE" % (label, obj.score),
                color=color,
            )
        return False

    border = geometry["border"]
    if config.SHOW_DEBUG_OVERLAYS:
        img.draw_rect(
            border["x"],
            border["y"],
            border["w"],
            border["h"],
            color=CYAN,
            thickness=1,
        )
        if geometry["shape_id"] == 1:
            corners = geometry["corners"]
            for index in range(4):
                p1 = corners[index]
                p2 = corners[(index + 1) % 4]
                img.draw_line(
                    int(p1[0]),
                    int(p1[1]),
                    int(p2[0]),
                    int(p2[1]),
                    color=YELLOW,
                    thickness=2,
                )

    raw_dx = geometry["cx"] - frame_cx
    dy = geometry["cy"] - frame_cy
    control_dx = 0 if abs(raw_dx) <= config.CENTER_DEAD_ZONE else raw_dx
    img.draw_cross(
        geometry["cx"], geometry["cy"], color=RED, size=8, thickness=2
    )

    if config.DEBUG_PRINT:
        print(
            "%s shape=%s center=(%d,%d) raw_dx=%+d dy=%+d "
            "control_dx=%+d q=%.2f fill=%.3f"
            % (
                label,
                geometry["shape"],
                geometry["cx"],
                geometry["cy"],
                raw_dx,
                dy,
                control_dx,
                geometry["score"],
                border["fill"],
            )
        )
    return True


def main():
    detector = nn.YOLO11(
        model=config.MODEL_PATH,
        dual_buff=config.DUAL_BUFF,
    )
    print("model:", config.MODEL_PATH)
    print("labels:", detector.labels)
    print("target class/shape:", config.TARGET_CLASS_ID, config.TARGET_SHAPE_ID)
    print(
        "input:",
        detector.input_width(),
        "x",
        detector.input_height(),
        detector.input_format(),
    )

    frame_cx = detector.input_width() // 2
    frame_cy = detector.input_height() // 2
    cam = camera.Camera(
        detector.input_width(),
        detector.input_height(),
        detector.input_format(),
    )
    disp = display.Display()

    while not app.need_exit():
        time.fps_start()
        img = cam.read()
        if img is None:
            continue

        objs = detector.detect(
            img,
            conf_th=config.CONF_TH,
            iou_th=config.IOU_TH,
        )
        selected_objs = select_objects(objs)

        # 先完成全部视觉计算再绘图，防止显示标记污染几何检测 ROI。
        results = []
        for obj in selected_objs:
            geometry, roi = find_geometry_center(img, obj)
            results.append((obj, geometry, roi))

        display_img = make_display_frame(img)
        draw_categories(display_img, detector, objs)
        geometry_count = 0
        for obj, geometry, roi in results:
            if draw_result(
                display_img,
                detector,
                obj,
                geometry,
                roi,
                frame_cx,
                frame_cy,
            ):
                geometry_count += 1

        if config.SHOW_DEBUG_OVERLAYS:
            display_img.draw_cross(
                frame_cx, frame_cy, color=GREEN, size=10, thickness=1
            )
        if config.SHOW_STATUS:
            display_img.draw_string(
                2,
                2,
                "FPS:%.1f det:%d geo:%d"
                % (time.fps(), len(objs), geometry_count),
                color=GREEN,
            )
        disp.show(display_img)


if __name__ == "__main__":
    main()
