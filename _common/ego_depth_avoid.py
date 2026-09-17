#!/usr/bin/env python3
"""轻量深度局部避障（教学版），思想对齐 Ego-Planner 感知链路。

完整仿真/规划栈请参考：
  https://github.com/Kinang2/Ego-Planner-System
  （深度 → grid_map 占据栅格 → A*/B 样条轨迹 → Offboard）

本模块不做完整栅格建图与轨迹优化，只在板端用深度图做：
  1) 深度滤波（近远距、边缘裁剪、skip_pixel，对齐 plan_env/grid_map 习惯）
  2) 多扇区自由距离评估（前/左前/右前/左/右 + 上/下）
  3) 目标偏向的反应式机体 FLU 速度（前/后/左/右/升/降）
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class EgoAvoidConfig:
    """反应式避障参数（深度滤波 + 碰撞阈值 + 速度上限）。

    Attributes:
        depth_min / depth_max: 有效深度范围（米），对齐 Ego grid_map 量级。
        skip_pixel: ROI 降采样步长。
        margin: 图像边缘裁剪像素。
        safe_distance: 进入减速/修正的前方距离。
        stop_distance: 急停/绕行阈值。
        max_vel: 机体速度幅值上限（室内台架很小）。
        enable_vertical / vertical_gain: 是否启用上下避让及增益。
    """
    # 对齐 Ego-Planner grid_map 深度滤波量级
    depth_min: float = 0.2
    depth_max: float = 5.0
    skip_pixel: int = 2
    margin: int = 2
    # 碰撞阈值
    safe_distance: float = 1.2
    stop_distance: float = 0.45
    # 速度
    max_vel: float = 0.02
    # 垂直避让增益（前方堵死且上下更开阔时启用）
    enable_vertical: bool = True
    vertical_gain: float = 0.35


@dataclass
class SectorClearance:
    """各扇区自由距离（米）；None 表示无效。"""
    front: float | None = None
    front_left: float | None = None
    front_right: float | None = None
    left: float | None = None
    right: float | None = None
    up: float | None = None
    down: float | None = None

    def as_dict(self) -> dict:
        """中文扇区名 → 距离，便于 HUD / 日志打印。"""
        return {
            '前': self.front,
            '左前': self.front_left,
            '右前': self.front_right,
            '左': self.left,
            '右': self.right,
            '上': self.up,
            '下': self.down,
        }


def _roi_clearance(depth_m: np.ndarray, x0, x1, y0, y1,
                   dmin: float, dmax: float, skip: int) -> float | None:
    """在归一化 ROI [x0,x1)×[y0,y1) 内取有效深度的 20% 分位（偏保守最近障碍）。

    有效像素过少时返回 None。
    """
    h, w = depth_m.shape[:2]
    xa, xb = int(w * x0), int(w * x1)
    ya, yb = int(h * y0), int(h * y1)
    if xb <= xa or yb <= ya:
        return None
    roi = depth_m[ya:yb:skip, xa:xb:skip]
    vals = roi[(roi > dmin) & (roi < dmax) & np.isfinite(roi)]
    if vals.size < 12:
        return None
    # 偏保守：取较低分位，接近 Ego 碰撞检查的“最近障碍”
    return float(np.percentile(vals, 20))


def measure_clearance(depth_m: np.ndarray, cfg: EgoAvoidConfig) -> SectorClearance:
    """多扇区自由距离（相机前视，图像左=机体左）。"""
    if depth_m is None or depth_m.size == 0:
        return SectorClearance()
    d = depth_m
    if cfg.margin > 0:
        m = int(cfg.margin)
        if d.shape[0] > 2 * m and d.shape[1] > 2 * m:
            d = d[m:-m, m:-m]
    skip = max(1, int(cfg.skip_pixel))
    dmin, dmax = cfg.depth_min, cfg.depth_max
    # 水平：中下视野（抑天空/近地面噪点），对齐教学例程习惯
    y0, y1 = 0.35, 0.78
    sc = SectorClearance(
        left=_roi_clearance(d, 0.02, 0.22, y0, y1, dmin, dmax, skip),
        front_left=_roi_clearance(d, 0.18, 0.42, y0, y1, dmin, dmax, skip),
        front=_roi_clearance(d, 0.38, 0.62, y0, y1, dmin, dmax, skip),
        front_right=_roi_clearance(d, 0.58, 0.82, y0, y1, dmin, dmax, skip),
        right=_roi_clearance(d, 0.78, 0.98, y0, y1, dmin, dmax, skip),
        up=_roi_clearance(d, 0.30, 0.70, 0.08, 0.32, dmin, dmax, skip),
        down=_roi_clearance(d, 0.30, 0.70, 0.72, 0.95, dmin, dmax, skip),
    )
    return sc


def _finite(v: float | None, fallback: float) -> float:
    """把 None / 非有限值替换为 ``fallback``。"""
    return float(v) if v is not None and math.isfinite(v) else fallback


def compute_body_velocity(
        depth_m: np.ndarray,
        goal_vx: float,
        goal_vy: float,
        cfg: EgoAvoidConfig,
) -> tuple[float, float, float, SectorClearance, str]:
    """目标速度 + 局部自由空间 → 机体 FLU (vx,vy,vz) 与说明。

    返回：(vx, vy, vz, clearance, message)
    message 为空表示未触发避障修正。
    """
    sc = measure_clearance(depth_m, cfg)
    front = sc.front
    if front is None:
        # 中带无效时用两侧较近者
        cands = [d for d in (sc.front_left, sc.front_right, sc.left, sc.right)
                 if d is not None]
        front = min(cands) if cands else None
        sc.front = front
    if front is None:
        return goal_vx, goal_vy, 0.0, sc, ''

    max_v = max(abs(cfg.max_vel), 1e-6)
    vx, vy, vz = float(goal_vx), float(goal_vy), 0.0

    # 过近：刹停，并向最开阔侧横移/后退
    if front <= cfg.stop_distance:
        fl = _finite(sc.front_left, front)
        fr = _finite(sc.front_right, front)
        l = _finite(sc.left, front)
        r = _finite(sc.right, front)
        # 选最开阔水平方向
        options = [
            (l, 0.0, max_v * 0.8, '左'),
            (r, 0.0, -max_v * 0.8, '右'),
            (fl, max_v * 0.15, max_v * 0.55, '左前'),
            (fr, max_v * 0.15, -max_v * 0.55, '右前'),
        ]
        options.sort(key=lambda t: t[0], reverse=True)
        best_d, bx, by, name = options[0]
        # 前方完全堵死时略后退
        if best_d < cfg.stop_distance * 1.2:
            bx = -max_v * 0.5
            name = '后+' + name
        msg = f'急停/绕行→{name} d={front:.2f}m'
        # 垂直：若上下明显更开阔
        if cfg.enable_vertical:
            u = _finite(sc.up, 0.0)
            dn = _finite(sc.down, 0.0)
            if max(u, dn) > front + 0.3:
                vz = (max_v * cfg.vertical_gain) if u >= dn else (-max_v * cfg.vertical_gain)
                msg += f'+{"升" if vz > 0 else "降"}'
        return bx, by, vz, sc, msg

    # 进入安全阈值：减速 + 向更开阔扇区修正
    if front < cfg.safe_distance:
        scale = max(
            0.0,
            (front - cfg.stop_distance)
            / max(cfg.safe_distance - cfg.stop_distance, 0.1))
        vx *= scale

        fl = _finite(sc.front_left, front)
        fr = _finite(sc.front_right, front)
        l = _finite(sc.left, front)
        r = _finite(sc.right, front)
        # 自由空间得分：距离越大越好；并偏向目标横向
        goal_left = 1.0 if goal_vy > 0 else (-1.0 if goal_vy < 0 else 0.0)
        scores = {
            '左': l + 0.15 * max(goal_left, 0) * cfg.safe_distance,
            '右': r + 0.15 * max(-goal_left, 0) * cfg.safe_distance,
            '左前': fl + 0.08 * max(goal_left, 0) * cfg.safe_distance,
            '右前': fr + 0.08 * max(-goal_left, 0) * cfg.safe_distance,
        }
        side = max(scores, key=scores.get)
        side_gain = max_v * 0.65 * (1.0 - scale)
        if side == '左':
            vy = max(vy, side_gain)
        elif side == '右':
            vy = min(vy, -side_gain)
        elif side == '左前':
            vy = max(vy, side_gain * 0.75)
            vx = max(vx, max_v * 0.2 * scale)
        else:
            vy = min(vy, -side_gain * 0.75)
            vx = max(vx, max_v * 0.2 * scale)

        fwd = '前' if vx > 1e-4 else ('后' if vx < -1e-4 else '刹前')
        msg = f'避障→{fwd}+{side} d={front:.2f}m'

        if cfg.enable_vertical:
            u = _finite(sc.up, front)
            dn = _finite(sc.down, front)
            # 前方仍紧、上下更开阔时轻微升降
            if front < cfg.safe_distance * 0.7 and max(u, dn) > front + 0.4:
                vz = (max_v * cfg.vertical_gain * 0.8) if u >= dn else (
                    -max_v * cfg.vertical_gain * 0.8)
                msg += f'+{"升" if vz > 0 else "降"}'
        return vx, vy, vz, sc, msg

    return vx, vy, 0.0, sc, ''
