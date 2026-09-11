#!/usr/bin/env python3
"""找画面里的停机坪 H 标（文档 4.2），不靠 COCO YOLO。

做法比较朴素：先缩小一点省 CPU，灰度模糊后用 Otsu / 霍夫圆捞候选框，
再在框里跟自己画的 H 模板做归一化相关（正色、反色都试）。分数最高且
过线的留下，坐标再映回原图。

返回 ``(x1, y1, x2, y2, score)``，找不到就 ``None``。score 越大越像 H。
"""
import cv2
import numpy as np

MIN_SIDE = 14              # 候选框最短边（缩小图坐标系）
MAX_FRAME_FRAC = 0.92      # 框不能几乎铺满整幅（排除整帧误检）
ASPECT_MIN = 0.40
ASPECT_MAX = 1.80
SCORE_MIN = 0.32
DETECT_MAX_WIDTH = 320     # 检测用缩略图宽；太大则帧率掉
MAX_PADS = 5               # 每帧最多评估的候选区


def h_template(height, width, thickness=None):
    """生成白底（0）上的白色 H 笔画模板，供 NCC 匹配。"""
    height = max(8, int(height))
    width = max(8, int(width))
    if thickness is None:
        thickness = max(2, int(round(min(height, width) * 0.22)))
    img = np.zeros((height, width), np.uint8)
    img[:, :thickness] = 255
    img[:, width - thickness:] = 255
    mid = height // 2
    half = max(1, thickness // 2)
    img[max(0, mid - half):min(height, mid + half + 1), :] = 255
    return img


def _ncc(a, b):
    """归一化互相关；尺寸不一致时把 b 缩到 a。"""
    if a.size < 16:
        return 0.0
    aw, ah = a.shape[1], a.shape[0]
    if b.shape[0] != ah or b.shape[1] != aw:
        b = cv2.resize(b, (aw, ah), interpolation=cv2.INTER_NEAREST)
    a32 = a.astype(np.float32)
    b32 = b.astype(np.float32)
    num = cv2.matchTemplate(a32, b32, cv2.TM_CCOEFF_NORMED)
    return float(num[0, 0]) if num.size else 0.0


def _score_bw(roi_bw):
    """ROI 与 H 模板的相似度（正色 / 反色取较大）。"""
    tmpl = h_template(roi_bw.shape[0], roi_bw.shape[1])
    return max(_ncc(roi_bw, tmpl), _ncc(255 - roi_bw, tmpl))


def _valid_box(w, h, frame_w, frame_h, min_side=MIN_SIDE):
    if w < min_side or h < min_side:
        return False
    if w > frame_w * MAX_FRAME_FRAC or h > frame_h * MAX_FRAME_FRAC:
        return False
    aspect = w / float(h)
    return ASPECT_MIN <= aspect <= ASPECT_MAX


def _boxes_from_binary(binary, frame_w, frame_h, min_side=MIN_SIDE):
    """二值图外轮廓 → 按面积排序的候选矩形列表。"""
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if not _valid_box(w, h, frame_w, frame_h, min_side):
            continue
        if cv2.contourArea(contour) < 0.12 * w * h:
            continue
        boxes.append((x, y, w, h, w * h))
    boxes.sort(key=lambda b: b[4], reverse=True)
    return [(x, y, w, h) for x, y, w, h, _ in boxes[:MAX_PADS]]


def _circle_boxes(gray):
    """霍夫圆：圆形停机坪外环也常包住 H。"""
    h, w = gray.shape[:2]
    max_r = max(16, min(h, w) // 2)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.5, minDist=max(24, min(h, w) // 5),
        param1=80, param2=32, minRadius=12, maxRadius=max_r)
    boxes = []
    if circles is None:
        return boxes
    for cx, cy, radius in np.round(circles[0, :3]).astype(int):
        r = int(radius * 1.05)
        x1 = max(0, cx - r)
        y1 = max(0, cy - r)
        x2 = min(w, cx + r)
        y2 = min(h, cy + r)
        if _valid_box(x2 - x1, y2 - y1, w, h, 12):
            boxes.append((x1, y1, x2 - x1, y2 - y1))
    return boxes


def _search_in_roi(gray_roi, ox, oy):
    """在单个候选区内找最佳 H；返回 (score,x1,y1,x2,y2) 相对整幅缩略图。"""
    rh, rw = gray_roi.shape[:2]
    if rh < MIN_SIDE or rw < MIN_SIDE:
        return None
    best = None
    _, otsu = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    for bw in (otsu, 255 - otsu):
        for x, y, w, h in _boxes_from_binary(bw, rw, rh, min_side=10):
            score = _score_bw(bw[y:y + h, x:x + w])
            item = (score, ox + x, oy + y, ox + x + w, oy + y + h)
            if best is None or score > best[0]:
                best = item
        if rw * rh <= 80 * 80:
            # 小 ROI：整块也当一次候选
            score_full = _score_bw(bw)
            item = (score_full, ox, oy, ox + rw, oy + rh)
            if best is None or score_full > best[0]:
                best = item
    return best


def detect_h_mark(frame_bgr, score_min=SCORE_MIN):
    """检测画面中的停机坪 H 标。

    Args:
        frame_bgr: OpenCV BGR 图
        score_min: 最低接受分数

    Returns:
        (x1, y1, x2, y2, score) 原图像素坐标，或 None
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return None
    full_h, full_w = frame_bgr.shape[:2]
    scale = 1.0
    small = frame_bgr
    if full_w > DETECT_MAX_WIDTH:
        scale = DETECT_MAX_WIDTH / float(full_w)
        small = cv2.resize(
            frame_bgr,
            (DETECT_MAX_WIDTH, max(1, int(round(full_h * scale)))),
            interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    sh, sw = gray.shape[:2]
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    pads = _boxes_from_binary(otsu, sw, sh)
    pads.extend(_boxes_from_binary(255 - otsu, sw, sh))
    pads.extend(_circle_boxes(blur))
    if not pads:
        pads.append((0, 0, sw, sh))  # 兜底：整幅搜索

    best = None
    seen = set()
    for x, y, w, h in pads[:MAX_PADS]:
        key = (x // 8, y // 8, w // 8, h // 8)
        if key in seen:
            continue
        seen.add(key)
        item = _search_in_roi(blur[y:y + h, x:x + w], x, y)
        if item is None:
            continue
        if best is None or item[0] > best[0]:
            best = item
    if best is None or best[0] < score_min:
        return None
    score, x1, y1, x2, y2 = best
    inv = 1.0 / scale
    return (
        int(round(x1 * inv)), int(round(y1 * inv)),
        int(round(x2 * inv)), int(round(y2 * inv)), float(score))
