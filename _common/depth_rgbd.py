#!/usr/bin/env python3
"""深度图 ↔ 点云与板端可视化（例程 09 / 10 共用）。

支持：
  - 真实深度话题（奥比中光 Orbbec / RealSense 等 ROS2 驱动）
  - ``simulate`` 合成房间深度，便于无相机时验证点云与导航可视化

点云在相机光学坐标系：x 右、y 下、z 前（与常见深度相机一致）。
板端不以 RViz 为必需：用 OpenCV 绘制俯视 / 侧视投影，并叠加飞行轨迹。
"""
from __future__ import annotations

import math
import time
from functools import lru_cache

import cv2
import numpy as np

try:
    from perf_utils import configure_runtime_threads
    configure_runtime_threads()
except Exception:
    pass

# 默认内参（640x480 量级）；订阅到 CameraInfo 后应覆盖
DEFAULT_FX = 385.0
DEFAULT_FY = 385.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0


def depth_msg_to_meters(depth_np, encoding: str) -> np.ndarray:
    """将 ROS 深度图转为 float32 米；无效处置 0。"""
    enc = (encoding or '').lower()
    if depth_np.dtype == np.float32 or '32fc' in enc:
        meters = depth_np.astype(np.float32)
    else:
        # 常见 16UC1：毫米
        meters = depth_np.astype(np.float32) * 0.001
    meters[~np.isfinite(meters)] = 0.0
    meters[meters < 0] = 0.0
    return meters


def colorize_depth(depth_m: np.ndarray, max_range: float = 4.0,
                   min_range: float = 0.2) -> np.ndarray:
    """深度伪彩。按有效像素 5%~95% 分位拉伸，避免整幅发蓝看不清。"""
    d = depth_m.astype(np.float32)
    valid = np.isfinite(d) & (d > float(min_range)) & (d <= float(max_range))
    color = np.full((*d.shape, 3), 18, np.uint8)
    if not np.any(valid):
        return color
    vals = d[valid]
    lo = float(np.percentile(vals, 5))
    hi = float(np.percentile(vals, 95))
    if hi - lo < 0.2:
        lo, hi = float(min_range), float(max_range)
    scaled = np.clip((d - lo) / (hi - lo + 1e-6), 0.0, 1.0)
    norm = (scaled * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    color[~valid] = (18, 18, 18)
    return color


@lru_cache(maxsize=32)
def _cached_rays(h: int, w: int, stride: int,
                 fx: float, fy: float, cx: float, cy: float):
    """Cache normalized camera rays; avoids rebuilding meshgrid every frame."""
    us = np.arange(0, w, stride, dtype=np.float32)
    vs = np.arange(0, h, stride, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs, indexing='xy')
    rx = (uu - np.float32(cx)) / np.float32(fx)
    ry = (vv - np.float32(cy)) / np.float32(fy)
    return rx, ry, uu.astype(np.int32), vv.astype(np.int32)


def depth_to_points(
        depth_m: np.ndarray,
        fx=DEFAULT_FX, fy=DEFAULT_FY, cx=DEFAULT_CX, cy=DEFAULT_CY,
        stride: int = 4, min_range: float = 0.4, max_range: float = 5.0,
        color_bgr: np.ndarray | None = None):
    """深度图反投影为相机系点云；热点路径完全 NumPy 向量化。"""
    h, w = depth_m.shape[:2]
    stride = max(1, int(stride))
    rx, ry, uu_i, vv_i = _cached_rays(
        h, w, stride, float(fx), float(fy), float(cx), float(cy))
    z = np.asarray(depth_m, dtype=np.float32)[::stride, ::stride]
    mask = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    if not np.any(mask):
        return np.zeros((0, 3), dtype=np.float32), None

    zv = z[mask]
    pts = np.empty((zv.size, 3), dtype=np.float32)
    pts[:, 0] = rx[mask] * zv
    pts[:, 1] = ry[mask] * zv
    pts[:, 2] = zv

    cols = None
    if color_bgr is not None and color_bgr.shape[:2] == depth_m.shape[:2]:
        cols = np.asarray(color_bgr)[::stride, ::stride][mask]
    return pts, cols


def clean_depth_m(depth_m: np.ndarray,
                  min_range: float = 0.4,
                  max_range: float = 5.0) -> np.ndarray:
    """深度后处理：中值 + 形态学开运算，去掉孤立噪点。"""
    d = depth_m.astype(np.float32).copy()
    d[~np.isfinite(d)] = 0.0
    d[(d > 0) & (d < min_range)] = 0.0
    d[d > max_range] = 0.0
    # 转成毫米 u16；保持一个中值 + 形态学阶段，避免每帧
    # connectedComponentsWithStats + Python 循环带来的额外 CPU/内存开销。
    mm = np.clip(d * 1000.0, 0, 65535).astype(np.uint16)
    if np.any(mm):
        mm = cv2.medianBlur(mm, 5)
        kernel = np.ones((3, 3), np.uint8)
        valid = cv2.morphologyEx(
            (mm > 0).astype(np.uint8), cv2.MORPH_OPEN, kernel, iterations=1)
        mm[valid == 0] = 0
    return mm.astype(np.float32) / 1000.0


def simulate_depth_room(
        width: int = 320, height: int = 240, t: float = 0.0,
        fx=DEFAULT_FX, fy=DEFAULT_FY, cx=None, cy=None,
        wall_dist: float = 2.5, corridor_half: float = 0.9,
        noise_std: float = 0.0) -> np.ndarray:
    """合成「前方走廊 + 左右墙」深度图，随时间轻微摆动便于观察点云变化。

    默认 320x240：板端模拟刷新更流畅；需要更密点云时可显式传更大分辨率。
    """
    if cx is None:
        cx = (width - 1) * 0.5
    if cy is None:
        cy = (height - 1) * 0.5
    # 内参随分辨率近似缩放（相对默认 640x480）
    sx = width / 640.0
    sy = height / 480.0
    fx_u = float(fx) * sx
    fy_u = float(fy) * sy

    sway = 0.15 * math.sin(t * 0.7)
    us = np.arange(width, dtype=np.float32)
    vs = np.arange(height, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs)
    xdir = (uu - float(cx)) / fx_u
    ydir = (vv - float(cy)) / fy_u
    depth = np.full((height, width), wall_dist, np.float32)
    front = wall_dist + sway
    with np.errstate(divide='ignore', invalid='ignore'):
        t_left = (-corridor_half) / xdir
        t_right = (corridor_half) / xdir
        t_floor = (0.6) / ydir
        t_ceil = (-0.6) / ydir
    for candidate in (t_left, t_right, t_floor, t_ceil):
        ok = np.isfinite(candidate) & (candidate > 0.2) & (candidate < depth)
        depth = np.where(ok, candidate.astype(np.float32), depth)
    depth = np.minimum(depth, front)
    if noise_std and noise_std > 0.0:
        depth = depth + (
            np.random.randn(height, width).astype(np.float32) * float(noise_std))
    return np.clip(depth, 0.2, 6.0)


def cam_to_body_flu(pts_cam: np.ndarray) -> np.ndarray:
    """相机光轴朝前、x 右 y 下 → 机体 FLU（前左上）。"""
    if pts_cam.size == 0:
        return pts_cam
    # body_x(前)=cam_z, body_y(左)=-cam_x, body_z(上)=-cam_y
    return np.stack([pts_cam[:, 2], -pts_cam[:, 0], -pts_cam[:, 1]], axis=1)


def body_to_enu(pts_body: np.ndarray, yaw: float, origin_enu) -> np.ndarray:
    """机体 FLU 点变换到 ENU（仅绕 z 的偏航）。"""
    if pts_body.size == 0:
        return pts_body
    c, s = math.cos(yaw), math.sin(yaw)
    x = c * pts_body[:, 0] - s * pts_body[:, 1]
    y = s * pts_body[:, 0] + c * pts_body[:, 1]
    z = pts_body[:, 2]
    ox, oy, oz = origin_enu
    return np.stack([x + ox, y + oy, z + oz], axis=1)


def _project_top(pts_enu, size=420, span=6.0):
    """ENU 俯视图：x 右、y 上（图像 y 向下故翻转）。"""
    img = np.zeros((size, size, 3), np.uint8)
    img[:] = (18, 18, 22)
    if pts_enu is None or len(pts_enu) == 0:
        return img
    scale = size / span
    cx = cy = size // 2
    xs = (pts_enu[:, 0] * scale + cx).astype(np.int32)
    ys = (cy - pts_enu[:, 1] * scale).astype(np.int32)
    mask = (xs >= 0) & (xs < size) & (ys >= 0) & (ys < size)
    xs, ys = xs[mask], ys[mask]
    if xs.size == 0:
        return img
    # 降采样后向量化写像素，避免 Python 逐点循环卡顿
    step = 1 if xs.size < 8000 else 2
    xs, ys = xs[::step], ys[::step]
    if pts_enu.shape[1] > 2:
        zs = pts_enu[mask, 2][::step]
    else:
        zs = np.zeros(len(xs), np.float32)
    # 高度编码为绿色；提亮便于台架小窗辨认
    g = np.clip(100.0 + zs * 70.0, 60.0, 255.0).astype(np.uint8)
    img[ys, xs, 0] = 30
    img[ys, xs, 1] = g
    img[ys, xs, 2] = 30
    cv2.line(img, (cx - 8, cy), (cx + 8, cy), (80, 80, 90), 1)
    cv2.line(img, (cx, cy - 8), (cx, cy + 8), (80, 80, 90), 1)
    # 标题由 render_modeling_panel 统一绘制，避免「TOP ENU」重影
    return img


def _rotate_enu_pts(pts, a: float):
    """ENU xy 绕原点逆时针旋转 a 弧度（点云/轨迹/航点统一换到显示系）。

    保持输入类型：list 进 list 出（tuple 元素），ndarray 进 ndarray 出。
    """
    if pts is None or len(pts) == 0:
        return pts
    c, s = math.cos(a), math.sin(a)
    arr = np.asarray(pts, dtype=np.float64)
    out = arr.copy()
    out[..., 0] = c * arr[..., 0] - s * arr[..., 1]
    out[..., 1] = s * arr[..., 0] + c * arr[..., 1]
    if isinstance(pts, list):
        return [tuple(row) for row in out.tolist()]
    return out


def _draw_polyline(img, pts_xy, color, thickness=2):
    size = img.shape[0]
    span = 6.0
    scale = size / span
    cx = cy = size // 2
    if pts_xy is None or len(pts_xy) < 2:
        return
    pix = []
    for p in pts_xy:
        x = int(p[0] * scale + cx)
        y = int(cy - p[1] * scale)
        if 0 <= x < size and 0 <= y < size:
            pix.append((x, y))
    for i in range(1, len(pix)):
        cv2.line(img, pix[i - 1], pix[i], color, thickness, cv2.LINE_AA)


def render_modeling_panel(
        depth_m: np.ndarray,
        pts_cam: np.ndarray,
        color_bgr: np.ndarray | None = None,
        trail_enu: list | None = None,
        planned_enu: list | None = None,
        pose_enu=None,
        yaw: float = 0.0,
        title: str = 'depth pointcloud',
        panel_mode: str = 'depth',
        north_yaw: float | None = None,
        markers: list | None = None) -> np.ndarray:
    """拼装可视化。

    panel_mode:
      depth — 仅深度伪彩（默认，OpenCV 图示）
      depth_cloud — 深度伪彩 | 俯视点云
      cloud — 仅俯视点云
      full  — 深度伪彩 | RGB | 俯视点云

    north_yaw: 给定时以该航向为「北」——整个场景绕 ENU 原点旋转，
    使该机头方向显示在屏幕正上方（N），默认 None 保持 ENU 正北朝上。

    markers: [(x, y, color_bgr, radius, shape)]，ENU 坐标目标标记
    （行人/跟随点等），shape ∈ {'circle','diamond','square'}，
    随 north_yaw 一起旋转投影到俯视图（无位姿时不画）。
    """
    mode = (panel_mode or 'depth').lower()
    depth_viz = colorize_depth(depth_m)

    if mode == 'depth':
        panel = depth_viz
        cv2.putText(panel, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(panel, f'points={len(pts_cam)}', (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return panel

    # 相机点 → 机体 → 以当前位姿为原点的局部 ENU（无位姿则用相机 z 前作俯视）
    if pose_enu is None:
        pts_plot = np.stack(
            [pts_cam[:, 0], pts_cam[:, 2], -pts_cam[:, 1]], axis=1
        ) if len(pts_cam) else pts_cam
        origin = (0.0, 0.0, 0.0)
        yaw_use = 0.0
    else:
        pts_body = cam_to_body_flu(pts_cam)
        pts_plot = body_to_enu(pts_body, yaw, pose_enu)
        origin = pose_enu
        yaw_use = yaw

    # 机头朝北：把锁定航向旋到屏幕正上方（无位姿时俯视本来就是前向朝上）
    rot = 0.0
    if north_yaw is not None and pose_enu is not None:
        rot = math.pi / 2.0 - float(north_yaw)
    if rot:
        pts_plot = _rotate_enu_pts(pts_plot, rot)
        if trail_enu:
            trail_enu = _rotate_enu_pts(trail_enu, rot)
        if planned_enu:
            planned_enu = _rotate_enu_pts(planned_enu, rot)

    top_size = 480 if mode == 'cloud' else min(360, max(240, depth_m.shape[0]))
    top = _project_top(pts_plot, size=top_size, span=6.0)
    if trail_enu:
        _draw_polyline(top, trail_enu, (0, 220, 255), 2)
    if planned_enu:
        _draw_polyline(top, planned_enu, (255, 180, 0), 2)
    if pose_enu is not None:
        size = top.shape[0]
        scale = size / 6.0
        cx = cy = size // 2
        c, s = math.cos(rot), math.sin(rot)
        rx = c * pose_enu[0] - s * pose_enu[1]
        ry = s * pose_enu[0] + c * pose_enu[1]
        px = int(rx * scale + cx)
        py = int(cy - ry * scale)
        # 净空底衬：机体标记永远位于点云团中心，先用背景色圆擦出净空区，
        # 再画红点/机头箭头，避免初始时刻被近距点云遮挡
        cv2.circle(top, (px, py), 22, (18, 18, 22), -1)
        cv2.circle(top, (px, py), 5, (0, 0, 255), -1)
        head = yaw_use + rot
        fx = int(px + 18 * math.cos(head))
        fy = int(py - 18 * math.sin(head))
        cv2.arrowedLine(top, (px, py), (fx, fy), (0, 0, 255), 2, tipLength=0.3)

    # 目标标记（行人/跟随点等）：随场景一起旋转投影到俯视图
    if markers and pose_enu is not None:
        size_m = top.shape[0]
        scale_m = size_m / 6.0
        cxm = cym = size_m // 2
        cm, sm = math.cos(rot), math.sin(rot)
        for mk in markers:
            mx, my, color, radius, shape = mk[:5]
            rx = cm * float(mx) - sm * float(my)
            ry = sm * float(mx) + cm * float(my)
            mxp = int(rx * scale_m + cxm)
            myp = int(cym - ry * scale_m)
            radius = max(3, int(radius))
            shape = (shape or 'circle').lower()
            if shape == 'diamond':
                pts = np.array([[mxp, myp - radius],
                                [mxp + radius, myp],
                                [mxp, myp + radius],
                                [mxp - radius, myp]], np.int32)
                cv2.fillPoly(top, [pts], color)
            elif shape == 'square':
                cv2.rectangle(top, (mxp - radius, myp - radius),
                              (mxp + radius, myp + radius), color, -1)
            else:
                cv2.circle(top, (mxp, myp), radius, color, -1)

    # N 指示：屏幕正上方即北（north_yaw 模式=初始机头方向；ENU 模式=正北）
    # 无位姿时俯视为相机系（前=上），不画 N
    if pose_enu is not None:
        cx_n = top.shape[1] // 2
        cv2.putText(top, 'N', (cx_n - 5, 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (235, 235, 245), 1, cv2.LINE_AA)
        cv2.arrowedLine(top, (cx_n, 38), (cx_n, 20), (210, 210, 225), 2,
                        tipLength=0.4)
    # 对齐 hobot_stereonet「3D Point」：OpenCV 俯视为主，不依赖 RViz 稠密点云
    top_label = '3D POINT (N=heading)' if rot else '3D POINT (TOP)'
    cv2.putText(top, top_label, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(top, f'points={len(pts_cam)}', (8, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(top, title[:40], (8, top.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

    if mode == 'cloud':
        _ = origin
        return top

    h = depth_viz.shape[0]
    top_r = cv2.resize(top, (h, h))
    if mode == 'depth_cloud':
        panel = np.hstack([depth_viz, top_r])
    else:
        if color_bgr is None:
            color_small = np.zeros_like(depth_viz)
            cv2.putText(color_small, 'no RGB', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (120, 120, 120), 2)
        else:
            color_small = cv2.resize(
                color_bgr, (depth_viz.shape[1], depth_viz.shape[0]))
        panel = np.hstack([depth_viz, color_small, top_r])
    if title:
        # title 为空时不画左上角标题/点数，保持深彩画面干净（数值见底部状态栏）
        cv2.putText(panel, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(panel, f'points={len(pts_cam)}', (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    _ = origin
    return panel


def points_to_cloud2_xyz(pts, stamp=None, frame_id: str = 'camera_depth_optical_frame'):
    """构造 sensor_msgs/PointCloud2（仅 xyz，兼容无 sensor_msgs_py 的板端）。"""
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    msg = PointCloud2()
    msg.header = Header()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(len(pts))
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    if len(pts) == 0:
        msg.data = []
    else:
        msg.data = np.asarray(pts, dtype=np.float32).reshape(-1).tobytes()
    return msg


class DepthSourceClock:
    """模拟源用单调时钟。"""

    def __init__(self):
        self._t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self._t0


def image_msg_to_bgr(msg, bridge=None) -> np.ndarray:
    """sensor_msgs/Image → BGR；支持 nv12（mipi_cam dual_combine 常用）。"""
    enc = (msg.encoding or '').lower()
    if enc in ('nv12', 'yuv420', 'yuv420p'):
        w, h = int(msg.width), int(msg.height)
        data = np.frombuffer(msg.data, dtype=np.uint8)
        expect = w * h * 3 // 2
        if data.size < expect:
            raise ValueError(f'nv12 data too short: {data.size} < {expect}')
        yuv = data[:expect].reshape((h * 3 // 2, w))
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    if bridge is None:
        from cv_bridge import CvBridge
        bridge = CvBridge()
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def split_stereo_combine(bgr: np.ndarray, layout: str = 'tb'):
    """拆分 dual_combine 拼接图。layout: tb=上下（左在上），lr=左右（左在左）。"""
    h, w = bgr.shape[:2]
    layout = (layout or 'tb').lower()
    if layout == 'lr':
        mid = w // 2
        return bgr[:, :mid].copy(), bgr[:, mid:mid * 2].copy()
    mid = h // 2
    return bgr[:mid].copy(), bgr[mid:mid * 2].copy()


def rotate_bgr(img: np.ndarray, rotate_cw: int = 0) -> np.ndarray:
    """顺时针旋转图像。rotate_cw ∈ {0, 90, 180, 270}。"""
    deg = int(rotate_cw) % 360
    if deg == 0 or img is None:
        return img
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError(f'unsupported rotate_cw={rotate_cw}')


def rotate_intrinsics_cw(fx, fy, cx, cy, width, height, rotate_cw: int = 0):
    """图像顺时针旋转后更新针孔内参。返回 (fx, fy, cx, cy, new_w, new_h)。"""
    deg = int(rotate_cw) % 360
    w, h = int(width), int(height)
    if deg == 0:
        return float(fx), float(fy), float(cx), float(cy), w, h
    if deg == 90:
        # (u,v)->(h-1-v, u)；尺寸变为 (h, w)
        return float(fy), float(fx), float(h - 1 - cy), float(cx), h, w
    if deg == 180:
        return float(fx), float(fy), float(w - 1 - cx), float(h - 1 - cy), w, h
    if deg == 270:
        return float(fy), float(fx), float(cy), float(w - 1 - cx), h, w
    raise ValueError(f'unsupported rotate_cw={rotate_cw}')


def stereo_to_depth_m(
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        fx: float,
        baseline_m: float = 0.06,
        max_width: int = 640,
        num_disparities: int = 128,
        block_size: int = 5,
        matcher: str = 'sgbm'):
    """左右目 → 深度（米）与缩放后的左目彩色。

    ``matcher``: ``sgbm``（更清晰，默认）或 ``bm``（更快更糙）。
    """
    if left_bgr is None or right_bgr is None:
        raise ValueError('left/right image required')
    if left_bgr.shape[:2] != right_bgr.shape[:2]:
        right_bgr = cv2.resize(right_bgr, (left_bgr.shape[1], left_bgr.shape[0]))

    h0, w0 = left_bgr.shape[:2]
    scale = 1.0
    if w0 > max_width > 0:
        scale = max_width / float(w0)
        nw = max_width
        nh = max(1, int(round(h0 * scale)))
        left = cv2.resize(left_bgr, (nw, nh), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right_bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    else:
        left, right = left_bgr, right_bgr

    fx_s = float(fx) * scale
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_l = clahe.apply(gray_l)
    gray_r = clahe.apply(gray_r)
    gray_l = cv2.GaussianBlur(gray_l, (3, 3), 0)
    gray_r = cv2.GaussianBlur(gray_r, (3, 3), 0)

    use_bm = str(matcher).lower() in ('bm', 'stereo_bm', 'fast')
    nd = max(16, int(num_disparities) // 16 * 16)
    if max_width <= 480:
        nd = min(nd, 96)
    else:
        nd = min(nd, 128)
    bs = int(block_size)
    if bs % 2 == 0:
        bs += 1
    bs = max(3, min(bs, 11))

    if use_bm:
        stereo = cv2.StereoBM_create(numDisparities=min(nd, 64), blockSize=max(bs, 9))
        stereo.setPreFilterCap(31)
        stereo.setMinDisparity(0)
        stereo.setUniquenessRatio(15)
        stereo.setSpeckleWindowSize(100)
        stereo.setSpeckleRange(2)
        stereo.setTextureThreshold(10)
        raw = stereo.compute(gray_l, gray_r)
        disp = raw.astype(np.float32) / 16.0
    else:
        # 3WAY 比 HH 更适合板端；HH 太慢
        mode = getattr(
            cv2, 'STEREO_SGBM_MODE_SGBM_3WAY', cv2.STEREO_SGBM_MODE_SGBM)
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=nd,
            blockSize=bs,
            P1=8 * 3 * bs * bs,
            P2=32 * 3 * bs * bs,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=120,
            speckleRange=2,
            preFilterCap=63,
            mode=mode,
        )
        raw = stereo.compute(gray_l, gray_r)
        disp = raw.astype(np.float32) / 16.0

    disp[disp < 0] = 0
    disp_u16 = np.clip(disp * 16.0, 0, 65535).astype(np.uint16)
    disp_u16 = cv2.medianBlur(disp_u16, 5)
    disp = disp_u16.astype(np.float32) / 16.0

    try:
        ximg = getattr(cv2, 'ximgproc', None)
        if ximg is not None and hasattr(ximg, 'createDisparityWLSFilterGeneric'):
            wls = ximg.createDisparityWLSFilterGeneric(False)
            wls.setLambda(8000.0)
            wls.setSigmaColor(1.2)
            filtered = wls.filter(raw, gray_l)
            disp = filtered.astype(np.float32) / 16.0
            disp[disp < 0] = 0
    except Exception:
        pass

    depth = np.zeros(disp.shape, np.float32)
    valid = disp > 1.5
    if float(baseline_m) > 1e-6 and fx_s > 1e-6:
        depth[valid] = (fx_s * float(baseline_m)) / disp[valid]
    depth = clean_depth_m(depth, min_range=0.4, max_range=5.0)
    return depth, left, fx_s, scale
