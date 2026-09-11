#!/usr/bin/env python3
"""
共享板端 YOLO 推理组件。文档 3.1 / 3.2。

BGR 帧 → letterbox 缩放 → NV12(h*w*1.5) → hbm_runtime.run
       → 反量化 → 三个尺度(8/16/32) DFL 解码 → 拼接 → 按类 NMS
       → 坐标映射回原图，返回像素框 [(x1, y1, x2, y2, score, cls_id), ...]。

解码链路与官方示例对齐
（/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8）。

被 04_object_detection / 05_obstacle_avoidance 共用。

用法（任务节点内）：
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
    from yolo_detector import YoloDetector

    det = YoloDetector(log=node.get_logger())   # 不传 log 则用 print
    if det.loaded:
        detections = det.detect(frame)

重要限制：
  - 640x640 NV12 输入是 h*w*1.5 字节，不能 reshape 成 (1,480,640,1)
  - 类别为 COCO 80 类；换模型需同步改类别表与解码假设（3 分支 / 16 分箱）
  - 无目标时返回空列表，禁止返回写死的假框
"""
import os

import numpy as np
import cv2

try:
    import hbm_runtime
except ImportError:
    hbm_runtime = None

# 按板上常见位置依次尝试
MODEL_CANDIDATES = [
    '/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin',
    '/opt/hobot/model/x5/basic/yolov8n_detect.bin',
    '/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/yolov8x_detect_bayese_640x640_nv12.bin',
]

# COCO 类别表，与官方示例同目录
CLASS_NAMES_CANDIDATES = [
    '/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/coco_classes.names',
    '/opt/hobot/model/x5/basic/coco_classes.names',
]

# 解码参数，与官方示例对齐
STRIDES = (8, 16, 32)      # 三个检测头的下采样步长
REG = 16                   # DFL 每个边（ltrb）的分箱数
RESIZE_TYPE = 1            # 1 = letterbox（保持比例 + 灰边），与 scale_coords_back 配套


class _PrintLog:
    """未传 logger 时的兜底输出。"""

    @staticmethod
    def info(msg):
        print(msg)

    @staticmethod
    def warn(msg):
        print(msg)

    @staticmethod
    def error(msg):
        print(msg)


def first_existing_model():
    for path in MODEL_CANDIDATES:
        if os.path.isfile(path):
            return path
    return MODEL_CANDIDATES[0]


def resolve_input_hw(shape):
    """从模型输入 shape 推断 (H, W)，兼容 NCHW 与 NHWC 两种声明。"""
    dims = [int(d) for d in shape]
    if len(dims) == 4:
        if dims[1] in (1, 3):       # (N, C, H, W)
            return dims[2], dims[3]
        return dims[1], dims[2]     # (N, H, W, C)
    if len(dims) == 3:              # (H, W, C)
        return dims[0], dims[1]
    raise ValueError(f'无法从输入 shape 推断 H/W: {shape}')


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x, axis):
    """数值稳定的 softmax，避免依赖 scipy。"""
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)


def gen_anchor(grid_size):
    """生成特征图网格中心（分辨率无关的归一化坐标，单位=格）。"""
    x = np.tile(np.linspace(0.5, grid_size - 0.5, grid_size), reps=grid_size)
    y = np.repeat(np.linspace(0.5, grid_size - 0.5, grid_size), grid_size)
    return np.stack([x, y], axis=1)


def resized_image(img, input_w, input_h, resize_type=RESIZE_TYPE):
    """缩放输入图：0=直接缩放，1=letterbox（保持比例 + 灰度 127 填充）。"""
    img_h, img_w = img.shape[:2]
    input_w = (input_w // 2) * 2     # NV12/YUV420 要求偶数
    input_h = (input_h // 2) * 2

    if resize_type == 0:
        return cv2.resize(img, (input_w, input_h))

    if resize_type != 1:
        raise ValueError(f'非法 resize_type: {resize_type}')

    scale = min(input_h / img_h, input_w / img_w)
    new_w, new_h = int(img_w * scale), int(img_h * scale)
    resized = cv2.resize(img, (new_w, new_h))

    pad_w = input_w - new_w
    pad_h = input_h - new_h
    left, right = pad_w // 2, pad_w - pad_w // 2
    top, bottom = pad_h // 2, pad_h - pad_h // 2
    return cv2.copyMakeBorder(resized, top, bottom, left, right,
                              borderType=cv2.BORDER_CONSTANT,
                              value=(127, 127, 127))


def bgr_to_nv12_planes(image):
    """BGR → NV12 的 Y / UV 平面，形状分别 (H, W) 与 (H/2, W/2, 2)。"""
    height, width = image.shape[:2]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area].reshape((height, width))
    u = yuv420p[area:area + area // 4].reshape((height // 2, width // 2))
    v = yuv420p[area + area // 4:].reshape((height // 2, width // 2))
    uv = np.stack((u, v), axis=-1)
    return y, uv


def is_scale_quant(quant_type):
    """判定量化类型是否为线性 scale（官方用 1 / SCALE 表示）。"""
    name = getattr(quant_type, 'name', None)
    if isinstance(name, str):
        return name.upper() == 'SCALE'
    return quant_type == 1


def dequantize_tensor(q_tensor, quant_info):
    """按量化参数反量化（支持 per-tensor / per-channel / 空 zero_point）。"""
    if not is_scale_quant(getattr(quant_info, 'quant_type', None)):
        return q_tensor.astype(np.float32, copy=False)

    scale = np.asarray(quant_info.scale, dtype=np.float32)
    zero_point = np.asarray(quant_info.zero_point, dtype=np.float32)
    q = q_tensor.astype(np.float32)

    if scale.ndim == 0 or q.ndim == 1 or scale.size == 1:
        zp = zero_point.reshape(-1)[0] if zero_point.size else np.float32(0.0)
        return (q - zp) * scale

    shape = [1] * q.ndim
    shape[getattr(quant_info, 'axis', 0)] = -1
    scale = scale.reshape(shape)
    # 对称量化的 zero_point 可能为空（等价于 0），不能直接 reshape
    if zero_point.size == 0:
        zero_point = np.zeros_like(scale)
    elif zero_point.size == 1:
        zero_point = np.full(shape, float(zero_point.reshape(-1)[0]),
                             dtype=np.float32)
    else:
        zero_point = zero_point.reshape(shape)
    return (q - zero_point) * scale


def nms(xyxy, score, cls, iou_thresh):
    """按类别逐类 NMS，返回保留的索引列表。"""
    keep = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]
        x1, y1, x2, y2 = xyxy[idx].T
        area = (x2 - x1) * (y2 - y1)
        order = score[idx].argsort()[::-1]
        while order.size > 0:
            i = order[0]
            keep.append(idx[i])
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
            iou = inter / (area[i] + area[order[1:]] - inter + 1e-9)
            order = order[1:][iou < iou_thresh]
    return keep


def scale_coords_back(xyxy, img_w, img_h, input_w, input_h,
                      resize_type=RESIZE_TYPE):
    """把输入尺寸下的坐标映射回原图并裁剪到图内。"""
    xyxy = xyxy.astype(np.float32).copy()
    if resize_type == 0:
        xyxy[:, [0, 2]] *= img_w / input_w
        xyxy[:, [1, 3]] *= img_h / input_h
    elif resize_type == 1:
        scale = min(input_w / img_w, input_h / img_h)
        pad_w = (input_w - img_w * scale) / 2
        pad_h = (input_h - img_h * scale) / 2
        xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad_w) / scale
        xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad_h) / scale
    else:
        raise ValueError('resize_type 只能是 0(resize) 或 1(letterbox)')

    xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, img_w)
    xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, img_h)
    return xyxy


class YoloDetector:
    """板端量化 YOLO 检测器：detect(bgr) → 像素框列表。

    log 需具备 .info / .warn / .error（rclpy logger 满足），
    不传则用 print。detect() 失败会抛异常，调用方自行兜底。
    """

    def __init__(self, score_thres=0.25, nms_thres=0.45,
                 model_path=None, log=None):
        self._log = log or _PrintLog()
        self.score_thres = float(score_thres)
        self.nms_thres = float(nms_thres)

        self.model = None
        self.model_name = None
        self.input_name = None
        self.output_names = []
        self.output_quants = {}
        self.input_h = 640
        self.input_w = 640
        self.anchor_sizes = [640 // s for s in STRIDES]
        self.anchors = {g: gen_anchor(g) for g in self.anchor_sizes}
        self.weights_static = np.arange(REG, dtype=np.float32)[
            np.newaxis, np.newaxis, :]
        self.class_names = self._load_class_names()

        if hbm_runtime is None:
            self._log.error('hbm_runtime 不可用，模型未加载')
        else:
            self._load_model(model_path or first_existing_model())

    @property
    def loaded(self):
        return self.model is not None

    def _load_class_names(self):
        for path in CLASS_NAMES_CANDIDATES:
            if os.path.isfile(path):
                try:
                    with open(path, 'r') as f:
                        names = [line.strip() for line in f if line.strip()]
                    if names:
                        return names
                except OSError as e:
                    self._log.warn(f'类别表读取失败（{path}）: {e}')
        return []

    def _load_model(self, model_path):
        try:
            model = hbm_runtime.HB_HBMRuntime(model_path)
            model_name = model.model_names[0]
            input_name = model.input_names[model_name][0]
            output_names = list(model.output_names[model_name])
            output_quants = model.output_quants[model_name]
            input_h, input_w = resolve_input_hw(
                model.input_shapes[model_name][input_name])

            if len(output_names) != 2 * len(STRIDES):
                raise ValueError(
                    f'输出分支数 {len(output_names)} 与本组件假设的 '
                    f'{2 * len(STRIDES)} 不一致，请核对 output_names')

            self.model = model
            self.model_name = model_name
            self.input_name = input_name
            self.output_names = output_names
            self.output_quants = output_quants
            self.input_h = input_h
            self.input_w = input_w
            self.anchor_sizes = [input_h // s for s in STRIDES]
            self.anchors = {g: gen_anchor(g) for g in self.anchor_sizes}
            self._log.info(f'已加载模型: {model_path} ({input_w}x{input_h})')
            self._log.info(f'输出张量: {output_names}')
        except Exception as e:
            self.model = None
            self._log.error(f'模型未加载，请改用 /app/pydev_demo 官方示例: {e}')

    def label(self, cls_id):
        if 0 <= cls_id < len(self.class_names):
            return self.class_names[cls_id]
        return f'cls{cls_id}'

    # ---- 推理链路 ----

    def _infer(self, frame):
        resized = resized_image(frame, self.input_w, self.input_h)
        y, uv = bgr_to_nv12_planes(resized)
        nv12 = np.concatenate((y.reshape(-1), uv.reshape(-1)), axis=0)
        nv12 = nv12.reshape((1, self.input_h * 3 // 2, self.input_w, 1))
        outputs = self.model.run({self.model_name: {self.input_name: nv12}})
        return outputs[self.model_name]

    def _decode_boxes(self, boxes_output, valid_indices, grid_size, stride):
        """DFL 解码：softmax(16 分箱) 求期望得 ltrb，再由网格中心还原 xyxy。"""
        boxes = boxes_output.reshape(-1, boxes_output.shape[-1]).astype(np.float32)
        picked = boxes[valid_indices]
        ltrb = np.sum(
            softmax(picked.reshape(-1, 4, REG), axis=2) * self.weights_static,
            axis=2)
        anchor = self.anchors[grid_size][valid_indices]
        x1y1 = anchor - ltrb[:, 0:2]
        x2y2 = anchor + ltrb[:, 2:4]
        return np.hstack([x1y1, x2y2]) * stride

    def _postprocess(self, outputs, img_w, img_h):
        """反量化 → 三尺度 DFL 解码 → 拼接 → 按类 NMS → 映射回原图。"""
        fp32_outputs = {
            name: dequantize_tensor(outputs[name], self.output_quants[name])
            for name in outputs}
        # 官方技巧：阈值反 sigmoid 到 logits 域，先粗筛再对少量格子做 sigmoid
        conf_thres_raw = -np.log(1.0 / self.score_thres - 1.0)

        all_boxes, all_scores, all_ids = [], [], []
        for i, stride in enumerate(STRIDES):
            cls_key = self.output_names[2 * i]
            box_key = self.output_names[2 * i + 1]

            cls_logits = fp32_outputs[cls_key].reshape(
                -1, fp32_outputs[cls_key].shape[-1])
            max_logits = cls_logits.max(axis=1)
            valid = np.flatnonzero(max_logits >= conf_thres_raw)
            if valid.size == 0:
                continue

            all_ids.append(np.argmax(cls_logits[valid], axis=1))
            all_scores.append(sigmoid(max_logits[valid]))
            all_boxes.append(
                self._decode_boxes(fp32_outputs[box_key], valid,
                                   self.anchor_sizes[i], stride))

        if not all_boxes:
            return []

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        ids = np.concatenate(all_ids, axis=0)

        keep = nms(boxes, scores, ids, self.nms_thres)
        xyxy = scale_coords_back(boxes[keep], img_w, img_h,
                                 self.input_w, self.input_h)
        return [(float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                 float(s), int(c)) for b, s, c in zip(xyxy, scores[keep], ids[keep])]

    def detect(self, frame_bgr):
        """BGR 帧 → 检测框列表 [(x1, y1, x2, y2, score, cls_id), ...]（像素坐标）。"""
        return self._postprocess(
            self._infer(frame_bgr), frame_bgr.shape[1], frame_bgr.shape[0])
