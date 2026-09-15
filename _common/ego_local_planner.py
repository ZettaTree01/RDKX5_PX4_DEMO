#!/usr/bin/env python3
"""Ego-Planner 风格局部占据地图 + A* 路径（教学板端实现）。

对齐 https://github.com/Kinang2/Ego-Planner-System 感知/规划话题形态：
  深度/点云 → 局部占据 → inflate → A* → 局部路径
完整 C++ EGO（B 样条优化）见该仓库；本模块在 RDK 上提供可运行的同构可视化与跟径控制。
"""
from __future__ import annotations

import heapq
import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np


@dataclass
class EgoMapConfig:
    resolution: float = 0.15          # 栅格分辨率 (m)，对齐 grid_map/resolution 量级
    local_range_xy: float = 4.0       # 局部更新半径
    local_range_z: float = 2.0
    inflation: float = 0.25           # 障碍膨胀
    max_voxels: int = 12000
    hit_count: int = 2                # 命中次数达此视为占据
    ground_z_min: float = -1.5
    ground_z_max: float = 2.0
    robot_radius: float = 0.20


class LocalOccupancyMap:
    """机体附近滚动占据体素（简化版 plan_env/grid_map）。"""

    def __init__(self, cfg: EgoMapConfig):
        self.cfg = cfg
        self._hits: OrderedDict[tuple[int, int, int], int] = OrderedDict()

    def clear(self):
        self._hits.clear()

    def _key(self, x, y, z):
        r = self.cfg.resolution
        return (int(math.floor(x / r)),
                int(math.floor(y / r)),
                int(math.floor(z / r)))

    def integrate_points(self, pts_map: np.ndarray, origin_xyz):
        """pts_map: Nx3 in map/world；仅保留 origin 附近局部窗。"""
        if pts_map is None or len(pts_map) == 0:
            return
        ox, oy, oz = origin_xyz
        rng = self.cfg.local_range_xy
        rz = self.cfg.local_range_z
        x = pts_map[:, 0]
        y = pts_map[:, 1]
        z = pts_map[:, 2]
        m = ((np.abs(x - ox) <= rng) & (np.abs(y - oy) <= rng)
             & (np.abs(z - oz) <= rz)
             & (z >= self.cfg.ground_z_min) & (z <= self.cfg.ground_z_max)
             & np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        pts = pts_map[m]
        if pts.size == 0:
            return
        # 下采样
        step = max(1, pts.shape[0] // 4000)
        pts = pts[::step]
        for p in pts:
            k = self._key(float(p[0]), float(p[1]), float(p[2]))
            self._hits[k] = self._hits.get(k, 0) + 1
            self._hits.move_to_end(k)
        while len(self._hits) > self.cfg.max_voxels:
            self._hits.popitem(last=False)

    def occupied_centers(self, inflated: bool = False) -> np.ndarray:
        r = self.cfg.resolution
        inf_n = int(math.ceil(self.cfg.inflation / r)) if inflated else 0
        cells = set()
        for (ix, iy, iz), c in self._hits.items():
            if c < self.cfg.hit_count:
                continue
            if inf_n <= 0:
                cells.add((ix, iy, iz))
            else:
                for dx in range(-inf_n, inf_n + 1):
                    for dy in range(-inf_n, inf_n + 1):
                        if dx * dx + dy * dy > inf_n * inf_n:
                            continue
                        cells.add((ix + dx, iy + dy, iz))
        if not cells:
            return np.zeros((0, 3), dtype=np.float32)
        arr = np.empty((len(cells), 3), dtype=np.float32)
        for i, (ix, iy, iz) in enumerate(cells):
            arr[i, 0] = (ix + 0.5) * r
            arr[i, 1] = (iy + 0.5) * r
            arr[i, 2] = (iz + 0.5) * r
        return arr

    def is_free_xy(self, x, y, z_ref, inflated: bool = True) -> bool:
        """2.5D：在 z_ref 附近检查 inflate 占据。"""
        r = self.cfg.resolution
        inf_n = int(math.ceil(
            (self.cfg.inflation + self.cfg.robot_radius) / r)) if inflated else 0
        ix0, iy0, iz0 = self._key(x, y, z_ref)
        for (ix, iy, iz), c in self._hits.items():
            if c < self.cfg.hit_count:
                continue
            if abs(iz - iz0) > 1:
                continue
            if abs(ix - ix0) <= inf_n and abs(iy - iy0) <= inf_n:
                if (ix - ix0) ** 2 + (iy - iy0) ** 2 <= inf_n * inf_n + 1e-6:
                    return False
        return True


def astar_plan_xy(
        occ: LocalOccupancyMap,
        start_xy,
        goal_xy,
        z_ref: float,
        max_expand: int = 4000,
) -> list[tuple[float, float, float]]:
    """平面 A*（对齐 path_searching/dyn_a_star 的局部搜索角色）。"""
    r = occ.cfg.resolution
    sx, sy = start_xy
    gx, gy = goal_xy
    start = (int(math.floor(sx / r)), int(math.floor(sy / r)))
    goal = (int(math.floor(gx / r)), int(math.floor(gy / r)))

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap = []
    heapq.heappush(open_heap, (h(start, goal), 0.0, start, None))
    came = {}
    gscore = {start: 0.0}
    closed = set()
    expands = 0
    neigh = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]

    found = None
    while open_heap and expands < max_expand:
        _, g, cur, parent = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        came[cur] = parent
        expands += 1
        if cur == goal or h(cur, goal) < 1.5:
            found = cur
            break
        for dx, dy in neigh:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt in closed:
                continue
            wx = (nxt[0] + 0.5) * r
            wy = (nxt[1] + 0.5) * r
            if not occ.is_free_xy(wx, wy, z_ref, inflated=True):
                continue
            step = 1.414 if dx and dy else 1.0
            ng = g + step
            if ng < gscore.get(nxt, 1e18):
                gscore[nxt] = ng
                heapq.heappush(
                    open_heap, (ng + h(nxt, goal), ng, nxt, cur))

    if found is None:
        # 失败：直线采样，供可视化与保守跟径
        return _line_fallback(sx, sy, gx, gy, z_ref, occ)

    # reconstruct
    path_cells = []
    c = found
    while c is not None:
        path_cells.append(c)
        c = came.get(c)
    path_cells.reverse()
    path = [((ix + 0.5) * r, (iy + 0.5) * r, z_ref) for ix, iy in path_cells]
    if not path:
        path = [(sx, sy, z_ref), (gx, gy, z_ref)]
    else:
        path[0] = (sx, sy, z_ref)
        path[-1] = (gx, gy, z_ref)
    return path


def _line_fallback(sx, sy, gx, gy, z, occ: LocalOccupancyMap):
    n = 12
    pts = []
    for i in range(n + 1):
        t = i / n
        x = sx + (gx - sx) * t
        y = sy + (gy - sy) * t
        if i > 0 and not occ.is_free_xy(x, y, z, inflated=True):
            break
        pts.append((x, y, z))
    if len(pts) < 2:
        pts = [(sx, sy, z), (sx, sy, z)]
    return pts


def path_follow_body_vel(
        path_xyz: list[tuple[float, float, float]],
        pose_xyz,
        yaw: float,
        max_vel: float,
        look_ahead: float = 0.45,
) -> tuple[float, float, float]:
    """沿规划路径跟径 → 机体 FLU 速度。"""
    if not path_xyz or pose_xyz is None:
        return 0.0, 0.0, 0.0
    px, py, pz = pose_xyz
    # 找前瞻点
    target = path_xyz[-1]
    best_d = 1e9
    for i, (x, y, z) in enumerate(path_xyz):
        d = math.hypot(x - px, y - py)
        if d < best_d:
            best_d = d
            # 前瞻索引
            j = min(i + max(1, int(look_ahead / max(0.05, 0.15))), len(path_xyz) - 1)
            target = path_xyz[j]
    tx, ty, tz = target
    dx, dy = tx - px, ty - py
    dist = math.hypot(dx, dy)
    if dist < 0.05:
        return 0.0, 0.0, 0.0
    vx_w = dx / dist * max_vel
    vy_w = dy / dist * max_vel
    c, s = math.cos(yaw), math.sin(yaw)
    vx_b = c * vx_w + s * vy_w
    vy_b = -s * vx_w + c * vy_w
    vz_b = float(np.clip(tz - pz, -max_vel * 0.4, max_vel * 0.4))
    return float(vx_b), float(vy_b), float(vz_b)
