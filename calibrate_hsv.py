# calibrate_hsv.py
# 运行环境: MaixPy v4 (MaixCam)
# 用途: 采用与 main.py 完全统一的 ROI 选区，自动检测并锁定区域内的红/绿色块，并提取色块内部像素的 HSV 特征，给出最精准的阈值推荐。

import cv2
import numpy as np
from maix import camera, display, image, app, time, pinmap, sys, gpio

def main():
    # 1. 初始化屏幕与相机
    disp = display.Display()
    cam = camera.Camera(320, 240)
    
    # 2. 手动设置曝光与增益 (与 main.py 保持完全一致)
    cam.exp_mode(camera.AeMode.Manual)
    cam.exposure(8000)  # 8ms
    cam.gain(80)
    
    # 3. 初始化并开启物理补光灯
    fill_light = None
    try:
        device = sys.device_id()
        if device == "maixcam2":
            pin_name = "B25"
            gpio_name = "GPIOB25"
        else:
            pin_name = "B3"
            gpio_name = "GPIOB3"
        
        pinmap.set_pin_function(pin_name, gpio_name)
        fill_light = gpio.GPIO(gpio_name, gpio.Mode.OUT)
        fill_light.value(1)
        print(f"[硬件] 开启补光灯成功 (引脚: {pin_name})")
    except Exception as e:
        print(f"[硬件] 开启补光灯失败: {e}")

    # 4. 配置与 main.py 统一的 ROI 参数与 Mask 构造
    w, h = 320, 240
    dead_x = int(w * 0.20)     # 64 像素
    dead_y = int(h * 0.10)     # 24 像素
    y_top = int(h * 0.28)      # 67 像素
    trunk_half = int(w * 0.09) # 28 像素
    cx = w // 2

    # 预先构造统一的 ROI 掩膜
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.rectangle(roi_mask, (cx - trunk_half, y_top), (cx + trunk_half, h), 255, -1)
    # 扣除盲区
    roi_mask[:, :dead_x] = 0
    roi_mask[:, w - dead_x:] = 0
    roi_mask[:dead_y, :] = 0

    print("====================================================")
    print(" 开启 ROI 自动锁定标定工具...")
    print(" 红色方框内为 main.py 统一的直道识别区域 (ROI)")
    print(" 将色块放入红色方框内，算法将自动锁定它并计算 HSV 阈值")
    print(" 按 Ctrl+C 可安全退出。")
    print("====================================================")

    try:
        while not app.need_exit():
            img_maix = cam.read()
            if img_maix is None:
                continue
            
            # 格式转换
            img_cv = image.image2cv(img_maix, copy=False)
            hsv_img = cv2.cvtColor(img_cv, cv2.COLOR_RGB2HSV)
            
            # 宽范围粗过滤色块 (HSV 宽容度很大，用于寻找轮廓)
            # 红色分两段处理
            red_m1 = cv2.inRange(hsv_img, np.array([0, 30, 25]), np.array([20, 255, 255]))
            red_m2 = cv2.inRange(hsv_img, np.array([155, 30, 25]), np.array([180, 255, 255]))
            red_mask = cv2.bitwise_or(red_m1, red_m2)
            
            # 绿色宽范围
            green_mask = cv2.inRange(hsv_img, np.array([25, 30, 25]), np.array([95, 255, 255]))
            
            # 仅限在统一的 ROI 内生效
            red_mask = cv2.bitwise_and(red_mask, roi_mask)
            green_mask = cv2.bitwise_and(green_mask, roi_mask)
            
            # 形态学滤波去噪
            kernel = np.ones((3, 3), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
            
            # 寻找轮廓
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 寻找 ROI 内面积最大的色块轮廓
            locked_color = None
            target_contour = None
            max_area = 0
            
            for c in red_contours:
                area = cv2.contourArea(c)
                if area > max_area and area > 100:
                    max_area = area
                    target_contour = c
                    locked_color = "red"
                    
            for c in green_contours:
                area = cv2.contourArea(c)
                if area > max_area and area > 100:
                    max_area = area
                    target_contour = c
                    locked_color = "green"
            
            # 始终绘制红色 ROI 物理识别框
            img_maix.draw_rect(cx - trunk_half, y_top, trunk_half * 2, h - y_top, image.Color.from_rgb(255, 0, 0), 1)
            
            if target_contour is not None:
                # 1. 绘制锁定框 (绿色)
                rx, ry, rw, rh = cv2.boundingRect(target_contour)
                img_maix.draw_rect(rx, ry, rw, rh, image.Color.from_rgb(0, 255, 255), 2)
                
                # 2. 构造色块掩膜，只提取该色块内部的 HSV 像素值
                c_mask = np.zeros_like(roi_mask)
                cv2.drawContours(c_mask, [target_contour], -1, 255, -1)
                pts = hsv_img[c_mask == 255]
                
                if len(pts) > 10:
                    h_vals = pts[:, 0]
                    s_vals = pts[:, 1]
                    v_vals = pts[:, 2]
                    
                    # 采用双侧 5% 分位数过滤边缘过渡噪点，获取最稳健的阈值区间
                    h_min, h_max = int(np.percentile(h_vals, 5)), int(np.percentile(h_vals, 95))
                    s_min, s_max = int(np.percentile(s_vals, 5)), int(np.percentile(s_vals, 95))
                    v_min, v_max = int(np.percentile(v_vals, 5)), int(np.percentile(v_vals, 95))
                    
                    # 3. 屏幕数据可视化渲染
                    img_maix.draw_string(10, 10, f"LOCKED: {locked_color.upper()}", image.Color.from_rgb(0, 255, 255), 1.2)
                    img_maix.draw_string(10, 30, f"Real H: {h_min} ~ {h_max}", image.Color.from_rgb(255, 255, 0), 1.0)
                    img_maix.draw_string(10, 48, f"Real S: {s_min} ~ {s_max}", image.Color.from_rgb(255, 255, 0), 1.0)
                    img_maix.draw_string(10, 66, f"Real V: {v_min} ~ {v_max}", image.Color.from_rgb(255, 255, 0), 1.0)
                    
                    # 4. 生成供 main.py 使用的阈值建议 (包含边缘松弛冗余)
                    rec_h_min = max(0, h_min - 2)
                    rec_h_max = min(180, h_max + 2)
                    rec_s_min = max(35, int(s_min * 0.80)) # S 轴松弛 20%
                    rec_v_min = max(35, int(v_min * 0.80)) # V 轴松弛 20%
                    
                    # 红色跨越 0/180 的特殊处理提示
                    if locked_color == "red":
                        img_maix.draw_string(10, 185, "Red uses 2 ranges (1 & 2)", image.Color.from_rgb(255, 100, 100), 0.9)
                    
                    img_maix.draw_string(10, 205, f"Suggest Lower: [{rec_h_min}, {rec_s_min}, {rec_v_min}]", image.Color.from_rgb(0, 255, 0), 1.0)
                    img_maix.draw_string(10, 220, f"Suggest Upper: [{rec_h_max}, 255, 255]", image.Color.from_rgb(0, 255, 0), 1.0)
                    
                    # 终端打印输出
                    if time.ticks_ms() % 300 < 30:
                        print(f"[锁定成功] 颜色: {locked_color} | 实际区间 H:{h_min}~{h_max}, S:{s_min}~{s_max}, V:{v_min}~{v_max}")
                        print(f" -> 建议 main.py 填入 Lower: [{rec_h_min}, {rec_s_min}, {rec_v_min}]")
            else:
                img_maix.draw_string(10, 10, "NO COLOR TARGET LOCKED", image.Color.from_rgb(255, 100, 100), 1.1)
                img_maix.draw_string(10, 30, "Please place target card inside RED box", image.Color.from_rgb(255, 255, 255), 0.9)

            # 显示结果
            disp.show(img_maix)
            
    except KeyboardInterrupt:
        pass
    finally:
        if fill_light:
            fill_light.value(0)
            print("[硬件] 补光灯已熄灭")
        print("[系统] 标定工具安全退出。")

if __name__ == "__main__":
    main()
