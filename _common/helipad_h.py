#!/usr/bin/env python3
"""停机坪 H 标识别（文档 4.2）。

提案（轮廓 / 霍夫圆）在 CPU 上做，很轻；候选 ROI 送到板端量化分类网络
（EfficientNet-lite / MobileNet，NV12 → BPU），与合成 H 模板的特征向量比对。
无 BPU 后端时回退归一化相关（NCC）。
"""
import os
from types import SimpleNamespace

import cv2
import numpy as np

MIN_SIDE = 14              # 候选框最短边（缩小图坐标系）
MAX_FRAME_FRAC = 0.92      # 框不能几乎铺满整幅（排除整帧误检）
ASPECT_MIN = 0.40
ASPECT_MAX = 1.80
SCORE_MIN = 0.32
DETECT_MAX_WIDTH = 320     # 检测用缩略图宽；太大则帧率掉
MAX_PADS = 5               # 每帧最多评估的候选区

_BPU_MODELS = (
    '/opt/hobot/model/x5/basic/efficientnet_lite0_224x224_nv12.bin',
    '/opt/hobot/model/x5/basic/mobilenetv2_224x224_nv12.bin',
    '/opt/hobot/model/x5/basic/mobilenetv1_224x224_nv12.bin',
)

try:
    import hobot_dnn.pyeasy_dnn as pyeasy_dnn
except ImportError:
    pyeasy_dnn = None

try:
    from yolo_detector import bgr_to_nv12_planes, resized_image, resolve_input_hw
except Exception:
    bgr_to_nv12_planes = resized_image = resolve_input_hw = None


class HelipadBpuScorer:
    """把 ROI 与合成 H 送到 BPU 分类网，用特征向量余弦相似度打分。"""

    def __init__(self):
        self.model = None
        self.input_h = 224
        self.input_w = 224
        self._ref = None
        path = next((p for p in _BPU_MODELS if os.path.isfile(p)), None)
        if path is None or pyeasy_dnn is None or resized_image is None:
            return
        try:
            models = pyeasy_dnn.load(path)
            if not models:
                return
            model = models[0]
            inp = model.inputs[0]
            self.input_h, self.input_w = resolve_input_hw(inp.properties.shape)
            self.model = model
            tmpl = cv2.cvtColor(h_template(self.input_h, self.input_w),
                                cv2.COLOR_GRAY2BGR)
            inv = cv2.cvtColor(255 - h_template(self.input_h, self.input_w),
                               cv2.COLOR_GRAY2BGR)
            self._ref = self._embed(tmpl)
            self._ref_inv = self._embed(inv)
        except Exception:
            self.model = None

    @property
    def loaded(self):
        """BPU 分类网是否可用。"""
        return self.model is not None and self._ref is not None

    def _nv12(self, bgr):
        """BGR → 模型 NV12 输入。"""
        resized = resized_image(bgr, self.input_w, self.input_h)
        y, uv = bgr_to_nv12_planes(resized)
        nv12 = np.concatenate((y.reshape(-1), uv.reshape(-1)), axis=0)
        return nv12.reshape((1, self.input_h * 3 // 2, self.input_w, 1))

    def _embed(self, bgr):
        """BPU 前向，返回一维特征向量。"""
        outs = self.model.forward(self._nv12(bgr))
        vec = np.asarray(outs[0].buffer, dtype=np.float32).reshape(-1)
        n = float(np.linalg.norm(vec) + 1e-6)
        return vec / n

    def score(self, bgr_roi):
        """ROI 与合成 H（正/反色）的最大余弦相似度，映射到 0~1。"""
        if not self.loaded or bgr_roi is None or bgr_roi.size < 16:
            return None
        try:
            emb = self._embed(bgr_roi)
        except Exception:
            return None
        sim = max(float(np.dot(emb, self._ref)),
                  float(np.dot(emb, self._ref_inv)))
        return max(0.0, min(1.0, 0.5 * (sim + 1.0)))


_BPU = SimpleNamespace(obj=None, tried=False)


def warmup_bpu():
    """启动时加载 BPU 分类网，避免第一帧卡住。"""
    if _BPU.tried:
        return _BPU.obj
    _BPU.tried = True
    scorer = HelipadBpuScorer()
    _BPU.obj = scorer if scorer.loaded else False
    return _BPU.obj


def bpu_ready():
    """H 标 BPU 打分是否已就绪。"""
    obj = warmup_bpu()
    return bool(obj)


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
    """候选框几何过滤：最短边、不铺满整幅、宽高比落在 H 标合理区间。"""
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
    scorer = warmup_bpu()
    for x, y, w, h in pads[:MAX_PADS]:
        key = (x // 8, y // 8, w // 8, h // 8)
        if key in seen:
            continue
        seen.add(key)
        item = _search_in_roi(blur[y:y + h, x:x + w], x, y)
        if item is None:
            continue
        score, x1, y1, x2, y2 = item
        if scorer:
            roi = small[max(0, y1):max(y1 + 1, y2), max(0, x1):max(x1 + 1, x2)]
            bpu_s = scorer.score(roi)
            if bpu_s is not None:
                score = 0.4 * float(score) + 0.6 * float(bpu_s)
        item = (score, x1, y1, x2, y2)
        if best is None or item[0] > best[0]:
            best = item
    if best is None:
        return None
    score, x1, y1, x2, y2 = best
    if score < score_min:
        return None
    inv = 1.0 / scale
    return (
        int(round(x1 * inv)), int(round(y1 * inv)),
        int(round(x2 * inv)), int(round(y2 * inv)), float(score))
