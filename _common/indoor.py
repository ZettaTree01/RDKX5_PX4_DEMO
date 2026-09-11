#!/usr/bin/env python3
"""室内调试限速，凡是会转电机的例程都从这里取值。

拆桨台架上希望能听出「怠速 → 干活加速 → 到顶」，又别飙到吓人转速。
速度、高度按实飞标称值乘 ``INDOOR_SPEED_SCALE``（默认 1/20）。

油门三段（0~1，再压到室内上限）：

  THR_MIN        刚转起来
  HOVER_THRUST   悬停
  THR_MAX        任务加速上限，大约对应 MAX_MOTOR_RPM（300）

加减速按大约 ``RAMP_SECONDS`` 秒从静到满来定，听得见变化。

要按实飞跑：把 ``INDOOR_SPEED_SCALE`` 改成 ``1.0``，或在 launch 里显式传
``altitude:=2 max_vel:=0.5`` 之类（任务节点会覆盖一部分限速）。
"""

# 总比例：0.05 = 实飞的 1/20。改 1.0 即全量实飞标称。
INDOOR_SPEED_SCALE = 0.05
MAX_MOTOR_RPM = 300       # 硬上限（有 ESC 遥测时还会再压速度指令）
RAMP_SECONDS = 2.0        # 起飞斜坡 / 加减速体感时间

# ---------- 实飞标称值（scale=1.0 时使用）----------
REAL_TAKEOFF_ALT_M = 2.0
REAL_AVOID_VEL_MPS = 0.5
REAL_TRACK_VEL_MPS = 1.0
REAL_CRUISE_SIDE_M = 1.0
REAL_FORMATION_SPAN_M = 2.0
REAL_BENCH_FOLLOW_MPS = 2.0
REAL_THR_MIN = 0.12
REAL_HOVER_THRUST = 0.50
REAL_THR_MAX = 1.00

# ---------- 室内导出量（任务 / 管理器 import 这些）----------
TAKEOFF_ALT_M = REAL_TAKEOFF_ALT_M * INDOOR_SPEED_SCALE
AVOID_VEL_MPS = REAL_AVOID_VEL_MPS * INDOOR_SPEED_SCALE
TRACK_VEL_MPS = REAL_TRACK_VEL_MPS * INDOOR_SPEED_SCALE
CRUISE_SIDE_M = REAL_CRUISE_SIDE_M * INDOOR_SPEED_SCALE
FORMATION_SPAN_M = REAL_FORMATION_SPAN_M * INDOOR_SPEED_SCALE
BENCH_FOLLOW_MPS = REAL_BENCH_FOLLOW_MPS * INDOOR_SPEED_SCALE

# 油门不能简单按 scale 压到 0.003，电调根本不转。
# 先封顶 THR_MAX≈0.025（约 300 r/min），再按实飞比例映射三段，并设下限。
THR_MAX = 0.025
_THR_SCALE = THR_MAX / REAL_THR_MAX
THR_MIN = max(0.012, REAL_THR_MIN * _THR_SCALE)
HOVER_THRUST = max(0.020, REAL_HOVER_THRUST * _THR_SCALE)
# 台架上速度太小就当没任务，改发位置悬停，电机还能听得见
BENCH_VEL_EPS = 0.001
MOTOR_TEST_VALUE = THR_MAX  # 01 电机测试默认输出

XY_VEL_MAX = max(TRACK_VEL_MPS, AVOID_VEL_MPS)
Z_VEL_MAX = max(AVOID_VEL_MPS, TAKEOFF_ALT_M / RAMP_SECONDS)
TKO_SPEED = max(0.03, TAKEOFF_ALT_M / RAMP_SECONDS)
ACC_HOR = max(0.02, XY_VEL_MAX / RAMP_SECONDS)
ACC_UP = max(0.02, Z_VEL_MAX / RAMP_SECONDS)
ACC_DOWN = ACC_UP
JERK_AUTO = max(0.05, ACC_HOR * 2.0)
LAND_SPEED = Z_VEL_MAX
BENCH_CLIMB_MPS = ACC_UP * 0.5


class RelAlt:
    """HUD 用的相对高度，免得室内气压计直接显示几十米。

    相对开机（或上次大跳变）时的 z0。相邻两次差超过 ``jump_m``，
    多半是视觉对齐或气压跳了一下，重新归零。
    """

    def __init__(self, jump_m=1.5):
        self.z0 = None
        self.raw = None
        self.jump_m = jump_m

    def update(self, z):
        """输入本地点 z（米），返回相对高度 z - z0。"""
        z = float(z)
        if self.z0 is None:
            self.z0 = z
        elif self.raw is not None and abs(z - self.raw) > self.jump_m:
            self.z0 = z
        self.raw = z
        return z - self.z0
