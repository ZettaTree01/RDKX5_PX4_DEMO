#!/usr/bin/env python3
"""轻量性能辅助，供各例程共用。

设计目标：
  - 不引入第三方依赖；
  - 避免无界帧队列堆积；
  - 一次性配置 OpenCV / NumPy 友好的线程数；
  - 辅助可选，各例程仍可独立运行。
"""
from __future__ import annotations

import os
import threading
from typing import Generic, Optional, TypeVar

T = TypeVar("T")


def configure_runtime_threads(cv_threads: int | None = None) -> None:
    """设置偏保守的 OpenCV 线程数，避免多节点 CPU 过订阅。

    在 RDK 类板端，多个 ROS2 节点若各自开 OpenCV 线程池会抢核。
    默认把 OpenCV 固定为较小线程池。
    环境变量 ``ZT_OPENCV_THREADS=0`` 表示不改动 OpenCV。

    Args:
        cv_threads: 显式线程数；``None`` 时读 ``ZT_OPENCV_THREADS``（默认 2）。
    """
    if cv_threads is None:
        raw = os.environ.get("ZT_OPENCV_THREADS", "2")
        try:
            cv_threads = int(raw)
        except ValueError:
            cv_threads = 2
    if cv_threads > 0:
        try:
            import cv2
            cv2.setNumThreads(cv_threads)
            try:
                cv2.setUseOptimized(True)
            except Exception:
                pass
        except Exception:
            pass


class LatestValue(Generic[T]):
    """线程安全的「只保留最新值」槽位。

    相机流通常应丢弃过期帧而非排队；本类刻意极简，无阻塞等待语义。

    Methods:
        put: 覆盖写入最新值。
        get: 读取当前值（可能为 None）。
    """
    __slots__ = ("_value", "_lock")

    def __init__(self, value: Optional[T] = None):
        """初始化；``value`` 为可选初始值。"""
        self._value = value
        self._lock = threading.Lock()

    def put(self, value: T) -> None:
        """写入最新值（覆盖旧值）。"""
        with self._lock:
            self._value = value

    def get(self) -> Optional[T]:
        """读取当前值；无数据时返回 None。"""
        with self._lock:
            return self._value
