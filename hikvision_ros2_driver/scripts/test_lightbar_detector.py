#!/usr/bin/env python3
"""
快速测试脚本：复现 hikvision_ros2_driver 的 OpenCV 装甲板检测逻辑。
用法示例：
  ./test_lightbar_detector.py --input test_images --outdir out
  ./test_lightbar_detector.py --input some.jpg --out some_out.jpg

输出：在控制台打印每张图是否检测到装甲板，并保存带框的结果到 --outdir 或指定 --out。
"""
import cv2
import numpy as np
import os
import argparse
from pathlib import Path


def detect_armor(img):
    # 输入 BGR 图像，返回 (armor_found, armor_box, vis_image)
    result = img.copy()

    kernel3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    kernel5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    kernel7 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    kernel9 = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 高亮区域（固定阈值）
    _, brightMask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
    brightMask = cv2.morphologyEx(brightMask, cv2.MORPH_OPEN, kernel7)
    brightMask = cv2.morphologyEx(brightMask, cv2.MORPH_CLOSE, kernel7)
    brightMask = cv2.morphologyEx(brightMask, cv2.MORPH_DILATE, kernel7)
    brightMask = cv2.morphologyEx(brightMask, cv2.MORPH_DILATE, kernel7)

    # HSV 蓝色区域（参数与 C++ 保持一致，可能需要调参）
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerBlue = np.array([160, 200, 200])
    upperBlue = np.array([180, 255, 255])
    blueMask = cv2.inRange(hsv, lowerBlue, upperBlue)

    # 组合掩码（C++ 中使用了 brightMask 作为最终掩码）
    combinedMask = brightMask  # 如果想也考虑蓝色，可以改为 cv2.bitwise_or(brightMask, blueMask)

    contours, _ = cv2.findContours(combinedMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_rects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 50:
            continue
        min_rect = cv2.minAreaRect(contour)  # ((cx,cy),(w,h),angle)
        (w, h) = min_rect[1]
        if w <= 0 or h <= 0:
            continue
        aspect_ratio = max(w, h) / min(w, h)
        if aspect_ratio < 1.5 or aspect_ratio > 5:
            continue

        angle = min_rect[2]
        # 保持与 C++ 相同的角度筛选逻辑
        if abs(angle) < 45 or abs(angle) > 135:
            continue

        rect_area = w * h
        compactness = area / rect_area if rect_area > 0 else 0
        if compactness < 0.3:
            continue

        valid_rects.append(min_rect)

    armor_found = False
    armor_box = None
    if len(valid_rects) >= 2:
        best_distance = 1e9
        best_i = -1
        best_j = -1
        for i in range(len(valid_rects)):
            for j in range(i + 1, len(valid_rects)):
                c1 = valid_rects[i][0]
                c2 = valid_rects[j][0]
                dist = np.linalg.norm(np.array(c1) - np.array(c2))
                y_diff = abs(c1[1] - c2[1])
                if y_diff > 100:
                    continue
                if dist < best_distance:
                    best_distance = dist
                    best_i = i
                    best_j = j
        if best_i != -1 and best_j != -1:
            rect1 = cv2.boxPoints(valid_rects[best_i])
            rect2 = cv2.boxPoints(valid_rects[best_j])
            all_pts = np.vstack((rect1, rect2))
            x, y, w, h = cv2.boundingRect(all_pts.astype(np.int32))
            armor_box = (x, y, w, h)
            armor_found = True
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return armor_found, armor_box, result


def process_path(input_path, outdir=None):
    p = Path(input_path)
    images = []
    if p.is_dir():
        for ext in ("*.jpg", "*.png", "*.jpeg", "*.bmp", "*.tiff"):
            images.extend(sorted(p.glob(ext)))
    elif p.is_file():
        images = [p]
    else:
        raise SystemExit(f"input path not found: {input_path}")

    if outdir:
        os.makedirs(outdir, exist_ok=True)

    for img_path in images:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Failed to read image: {img_path}")
            continue
        found, box, vis = detect_armor(img)
        print(f"{img_path.name}: armor_found={found}, box={box}")
        if outdir:
            out_path = Path(outdir) / (img_path.stem + "_out.png")
            cv2.imwrite(str(out_path), vis)
        else:
            cv2.imshow(str(img_path), vis)
            key = cv2.waitKey(0)
            if key == 27:  # ESC
                break
            cv2.destroyWindow(str(img_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test lightbar/armor detection (OpenCV)')
    parser.add_argument('--input', '-i', required=True, help='Input image or directory')
    parser.add_argument('--outdir', '-o', default=None, help='Output directory to save results (if omitted, show windows)')
    args = parser.parse_args()
    process_path(args.input, args.outdir)
