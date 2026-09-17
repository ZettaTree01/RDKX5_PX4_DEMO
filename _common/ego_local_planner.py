#!/usr/bin/env python3
"""性能优化版：局部体素占据 + 2.5D A*。

核心优化：
1. 点云体素化使用 NumPy unique/count，移除逐点 Python 循环；
2. 查询使用 occupied set，A* 不再扫描全部体素；
3. inflation 偏移预计算；
4. 保留原有公开类/函数接口，便于例程直接替换。
"""
from __future__ import annotations

import heapq
import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np


@dataclass
class EgoMapConfig:
    resolution: float = 0.15
    local_range_xy: float = 4.0
    local_range_z: float = 2.0
    inflation: float = 0.25
    max_voxels: int = 12000
    hit_count: int = 2
    ground_z_min: float = -1.5
    ground_z_max: float = 2.0
    robot_radius: float = 0.20


class LocalOccupancyMap:
    def __init__(self, cfg: EgoMapConfig):
        self.cfg = cfg
        self._hits: OrderedDict[tuple[int, int, int], int] = OrderedDict()
        self._occupied: set[tuple[int, int, int]] = set()
        self._inflated_cache: dict[tuple[int, int], set[tuple[int, int, int]]] = {}
        self._offset_cache: dict[int, tuple[tuple[int, int], ...]] = {}

    def clear(self):
        self._hits.clear()
        self._occupied.clear()
        self._inflated_cache.clear()

    def _key(self, x, y, z):
        r = self.cfg.resolution
        return (int(math.floor(x / r)),
                int(math.floor(y / r)),
                int(math.floor(z / r)))

    def integrate_points(self, pts_map: np.ndarray, origin_xyz):
        """向量化体素化；仅在最终唯一体素上更新命中计数。"""
        if pts_map is None or len(pts_map) == 0:
            return
        pts_map = np.asarray(pts_map, dtype=np.float32)
        ox, oy, oz = origin_xyz
        x, y, z = pts_map[:, 0], pts_map[:, 1], pts_map[:, 2]
        rng, rz = self.cfg.local_range_xy, self.cfg.local_range_z
        m = ((np.abs(x - ox) <= rng) & (np.abs(y - oy) <= rng)
             & (np.abs(z - oz) <= rz)
             & (z >= self.cfg.ground_z_min) & (z <= self.cfg.ground_z_max)
             & np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        pts = pts_map[m]
        if pts.size == 0:
            return

        # 上限采样，避免极密 PointCloud2 在板端造成无意义 CPU 消耗。
        if pts.shape[0] > 6000:
            idx = np.linspace(0, pts.shape[0] - 1, 6000, dtype=np.int32)
            pts = pts[idx]

        inv = 1.0 / self.cfg.resolution
        cells = np.floor(pts * inv).astype(np.int32)
        unique_cells, counts = np.unique(cells, axis=0, return_counts=True)

        changed = False
        for cell, count in zip(unique_cells, counts):
            k = (int(cell[0]), int(cell[1]), int(cell[2]))
            old = self._hits.get(k, 0)
            # 每帧只算一次“观测命中”，而不是同一帧点数累加到阈值。
            new = min(self.cfg.hit_count, old + 1)
            self._hits[k] = new
            self._hits.move_to_end(k)
            if old < self.cfg.hit_count <= new:
                self._occupied.add(k)
                changed = True

        while len(self._hits) > self.cfg.max_voxels:
            k, _ = self._hits.popitem(last=False)
            self._occupied.discard(k)
            changed = True
        if changed:
            self._inflated_cache.clear()

    def _inflation_offsets(self, inf_n: int):
        offsets = self._offset_cache.get(inf_n)
        if offsets is None:
            offsets = tuple(
                (dx, dy)
                for dx in range(-inf_n, inf_n + 1)
                for dy in range(-inf_n, inf_n + 1)
                if dx * dx + dy * dy <= inf_n * inf_n
            )
            self._offset_cache[inf_n] = offsets
        return offsets

    def occupied_centers(self, inflated: bool = False) -> np.ndarray:
        r = self.cfg.resolution
        if not inflated:
            cells = self._occupied
        else:
            inf_n = int(math.ceil(self.cfg.inflation / r))
            cache_key = (inf_n, len(self._occupied))
            cells = self._inflated_cache.get(cache_key)
            if cells is None:
                cells = set()
                offsets = self._inflation_offsets(inf_n)
                for ix, iy, iz in self._occupied:
                    for dx, dy in offsets:
                        cells.add((ix + dx, iy + dy, iz))
                self._inflated_cache[cache_key] = cells

        if not cells:
            return np.zeros((0, 3), dtype=np.float32)
        arr = np.fromiter(cells, dtype=np.int32).reshape(-1, 3).astype(np.float32)
        arr = (arr + 0.5) * np.float32(r)
        return arr

    def is_free_xy(self, x, y, z_ref, inflated: bool = True) -> bool:
        r = self.cfg.resolution
        inf_n = int(math.ceil(
            (self.cfg.inflation + self.cfg.robot_radius) / r)) if inflated else 0
        ix0, iy0, iz0 = self._key(x, y, z_ref)
        # 2.5D：检查当前 z 层及上下相邻两层。
        for dz in (-1, 0, 1):
            iz = iz0 + dz
            for dx, dy in self._inflation_offsets(inf_n):
                if (ix0 + dx, iy0 + dy, iz) in self._occupied:
                    return False
        return True


def astar_plan_xy(occ: LocalOccupancyMap, start_xy, goal_xy,
                  z_ref: float, max_expand: int = 4000):
    r = occ.cfg.resolution
    sx, sy = start_xy
    gx, gy = goal_xy
    start = (int(math.floor(sx / r)), int(math.floor(sy / r)))
    goal = (int(math.floor(gx / r)), int(math.floor(gy / r)))

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    if not occ.is_free_xy(sx, sy, z_ref):
        return []
    if not occ.is_free_xy(gx, gy, z_ref):
        return []

    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))
    open_heap = [(h(start, goal), 0.0, start)]
    came = {}
    gscore = {start: 0.0}
    closed = set()
    expands = 0

    while open_heap and expands < max_expand:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expands += 1
        if cur == goal:
            break

        for dx, dy in neigh:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt in closed:
                continue
            x = (nxt[0] + 0.5) * r
            y = (nxt[1] + 0.5) * r
            if not occ.is_free_xy(x, y, z_ref):
                continue
            step = 1.41421356237 if dx and dy else 1.0
            ng = gc + step
            if ng < gscore.get(nxt, float('inf')):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_heap, (ng + h(nxt, goal), ng, nxt))

    if goal not in came and goal != start:
        return []
    path_cells = [goal]
    while path_cells[-1] != start:
        path_cells.append(came[path_cells[-1]])
    path_cells.reverse()
    return [((ix + 0.5) * r, (iy + 0.5) * r, z_ref)
            for ix, iy in path_cells]


def _line_fallback(sx, sy, gx, gy, z, occ):
    n = max(2, int(math.hypot(gx - sx, gy - sy) / occ.cfg.resolution))
    for i in range(n + 1):
        t = i / n
        x, y = sx + (gx - sx) * t, sy + (gy - sy) * t
        if not occ.is_free_xy(x, y, z):
            return []
    return [(sx, sy, z), (gx, gy, z)]


def path_follow_body_vel(path_xyz, pose_xyz, yaw, max_vel):
    if not path_xyz:
        return 0.0, 0.0, 0.0
    px, py, pz = pose_xyz
    # 找到最近点后向前看少量路径点；避免每次扫描整条长路径。
    best_i = 0
    best_d2 = float('inf')
    start_i = max(0, len(path_xyz) - 80)
    for i in range(start_i, len(path_xyz)):
        dx, dy = path_xyz[i][0] - px, path_xyz[i][1] - py
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_i, best_d2 = i, d2
    target_i = min(len(path_xyz) - 1, best_i + 3)
    tx, ty, tz = path_xyz[target_i]
    dx, dy = tx - px, ty - py
    c, s = math.cos(yaw), math.sin(yaw)
    vx = c * dx + s * dy
    vy = -s * dx + c * dy
    scale = max(1.0, math.hypot(vx, vy) / max(max_vel, 1e-6))
    return vx / scale, vy / scale, (tz - pz) * 0.5
