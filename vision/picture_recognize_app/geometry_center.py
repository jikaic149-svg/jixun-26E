"""YOLO ROI 内的低开销深色外框与严格几何中心计算。"""

import math

import vision_config as config


def clamp(value, low, high):
    return max(low, min(high, value))


def read_member(item, name, default=None):
    """兼容 MaixPy 成员是属性或无参方法的两种形式。"""
    try:
        value = getattr(item, name)
        return value() if callable(value) else value
    except Exception:
        return default


def ratio_score(a, b):
    a = max(1.0, float(a))
    b = max(1.0, float(b))
    return min(a, b) / max(a, b)


def make_center_roi(box, scale, img_w, img_h):
    x, y, width, height = box
    roi_w = min(img_w, max(8, int(width * scale)))
    roi_h = min(img_h, max(8, int(height * scale)))
    center_x = x + width * 0.5
    center_y = y + height * 0.5
    roi_x = clamp(int(center_x - roi_w * 0.5), 0, img_w - roi_w)
    roi_y = clamp(int(center_y - roi_h * 0.5), 0, img_h - roi_h)
    return [int(roi_x), int(roi_y), int(roi_w), int(roi_h)]


def make_square_roi(box, scale, img_w, img_h):
    """以 YOLO 框长边为正方形边长，保持中心不变后再按比例扩大。"""
    x, y, width, height = box
    side = max(8, int(max(width, height) * scale))
    roi_w = min(img_w, side)
    roi_h = min(img_h, side)
    center_x = x + width * 0.5
    center_y = y + height * 0.5
    roi_x = clamp(int(center_x - roi_w * 0.5), 0, img_w - roi_w)
    roi_y = clamp(int(center_y - roi_h * 0.5), 0, img_h - roi_h)
    return [int(roi_x), int(roi_y), int(roi_w), int(roi_h)]


def blob_values(blob):
    try:
        x, y = int(blob[0]), int(blob[1])
        width, height = int(blob[2]), int(blob[3])
    except Exception:
        x = int(read_member(blob, "x", 0))
        y = int(read_member(blob, "y", 0))
        width = int(read_member(blob, "w", 0))
        height = int(read_member(blob, "h", 0))
    pixels = int(read_member(blob, "pixels", width * height))
    blob_cx = float(read_member(blob, "cx", x + width * 0.5))
    blob_cy = float(read_member(blob, "cy", y + height * 0.5))
    return x, y, width, height, pixels, blob_cx, blob_cy


def scan_dark_outer_border(img, obj, roi):
    """在给定 ROI 内寻找尺寸、中心和填充率最匹配的连续深色外框。"""
    object_area = max(1, obj.w * obj.h)
    pixels_threshold = max(10, object_area // 900)
    area_threshold = max(20, object_area // 24)

    try:
        blobs = img.find_blobs(
            config.DARK_BORDER_THRESHOLDS,
            roi=roi,
            pixels_threshold=pixels_threshold,
            area_threshold=area_threshold,
            merge=False,
        )
    except Exception as error:
        if config.DEBUG_PRINT:
            print("find_blobs failed:", error)
        return None

    yolo_cx = obj.x + obj.w * 0.5
    yolo_cy = obj.y + obj.h * 0.5
    yolo_diag = max(1.0, math.hypot(obj.w, obj.h))
    best = None

    for blob in blobs or ():
        x, y, width, height, pixels, blob_cx, blob_cy = blob_values(blob)
        if width < max(6, obj.w * 0.60) or height < max(6, obj.h * 0.60):
            continue
        if width > obj.w * 2.05 or height > obj.h * 2.05:
            continue

        center_x = x + width * 0.5
        center_y = y + height * 0.5
        distance = math.hypot(center_x - yolo_cx, center_y - yolo_cy)
        center_score = max(0.0, 1.0 - distance / (yolo_diag * 0.50))
        size_score = 0.5 * (
            ratio_score(width, obj.w) + ratio_score(height, obj.h)
        )
        fill = pixels / max(1.0, width * height)
        fill_score = max(0.0, 1.0 - abs(fill - 0.18) / 0.35)
        touches_roi = (
            x <= roi[0] + 1
            or y <= roi[1] + 1
            or x + width >= roi[0] + roi[2] - 1
            or y + height >= roi[1] + roi[3] - 1
        )
        enclosure_score = 0.15 if touches_roi else 1.0
        score = (
            0.45 * center_score
            + 0.40 * size_score
            + 0.10 * fill_score
            + 0.05 * enclosure_score
        )

        if best is None or score > best["score"]:
            best = {
                "x": x,
                "y": y,
                "w": width,
                "h": height,
                "cx": int(round(center_x)),
                "cy": int(round(center_y)),
                "blob_cx": int(round(blob_cx)),
                "blob_cy": int(round(blob_cy)),
                "fill": fill,
                "score": score,
                "touches_roi": touches_roi,
                "roi": roi,
            }

    if best is None or best["score"] < config.BORDER_MIN_SCORE:
        return None
    return best


def find_dark_outer_border(img, obj):
    """先搜索1.20倍长边正方形，疑似截断时才回退到1.70倍。"""
    box = (obj.x, obj.y, obj.w, obj.h)
    primary_roi = make_square_roi(
        box, config.BORDER_PRIMARY_SCALE, img.width(), img.height()
    )
    primary = scan_dark_outer_border(img, obj, primary_roi)
    if primary is not None and not primary["touches_roi"]:
        return primary, primary_roi

    fallback_roi = make_square_roi(
        box, config.BORDER_FALLBACK_SCALE, img.width(), img.height()
    )
    fallback = scan_dark_outer_border(img, obj, fallback_roi)
    if fallback is not None:
        return fallback, fallback_roi
    return None, fallback_roi


def polygon_area(points):
    total = 0.0
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def order_quad(points):
    if points is None or len(points) != 4:
        return None
    result = [(float(point[0]), float(point[1])) for point in points]
    center_x = sum(point[0] for point in result) * 0.25
    center_y = sum(point[1] for point in result) * 0.25
    result.sort(
        key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x)
    )
    return result


def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-6:
        return None
    cross12 = x1 * y2 - y1 * x2
    cross34 = x3 * y4 - y3 * x4
    center_x = (cross12 * (x3 - x4) - (x1 - x2) * cross34) / denominator
    center_y = (cross12 * (y3 - y4) - (y1 - y2) * cross34) / denominator
    return int(round(center_x)), int(round(center_y))


def find_rectangle_center(img, border):
    """只在目标形状明确为矩形时运行一次局部四角检测。"""
    roi = make_center_roi(
        (border["x"], border["y"], border["w"], border["h"]),
        1.08,
        img.width(),
        img.height(),
    )
    try:
        rects = img.find_rects(roi=roi, threshold=config.RECT_THRESHOLD)
    except Exception as error:
        if config.DEBUG_PRINT:
            print("find_rects failed:", error)
        return None

    best = None
    for rect in rects or ():
        corners = order_quad(read_member(rect, "corners"))
        if corners is None:
            continue
        center = line_intersection(corners[0], corners[2], corners[1], corners[3])
        if center is None:
            continue
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        area = polygon_area(corners)
        size_score = 0.5 * (
            ratio_score(width, border["w"]) + ratio_score(height, border["h"])
        )
        distance = math.hypot(center[0] - border["cx"], center[1] - border["cy"])
        center_score = max(
            0.0,
            1.0
            - distance
            / max(1.0, math.hypot(border["w"], border["h"]) * 0.4),
        )
        area_score = min(1.0, area / max(1.0, border["w"] * border["h"]))
        score = 0.50 * size_score + 0.40 * center_score + 0.10 * area_score
        if best is None or score > best["score"]:
            best = {
                "shape": "RECT",
                "shape_id": 1,
                "cx": center[0],
                "cy": center[1],
                "score": min(border["score"], score),
                "corners": corners,
                "border": border,
            }
    return best


def find_geometry_center(img, obj):
    """按已知目标形状选择最低开销的几何中心算法。"""
    border, roi = find_dark_outer_border(img, obj)
    if border is None:
        return None, roi

    if config.TARGET_SHAPE_ID == 1:
        rectangle = find_rectangle_center(img, border)
        if rectangle is not None:
            rectangle["shape"] = "SQUARE"
        return rectangle, roi

    if config.TARGET_SHAPE_ID == 2:
        return {
            "shape": "CIRCLE",
            "shape_id": 2,
            "cx": border["cx"],
            "cy": border["cy"],
            "score": border["score"],
            "corners": None,
            "border": border,
        }, roi

    rectangle = find_rectangle_center(img, border)
    if rectangle is not None:
        rectangle["shape"] = "SQUARE"
        return rectangle, roi
    return {
        "shape": "CIRCLE",
        "shape_id": 2,
        "cx": border["cx"],
        "cy": border["cy"],
        "score": border["score"],
        "corners": None,
        "border": border,
    }, roi
