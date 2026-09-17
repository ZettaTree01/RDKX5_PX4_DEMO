#!/usr/bin/env python3
"""Lightweight performance helpers shared by the demo examples.

Design goals:
- do not add third-party dependencies;
- avoid unbounded frame queues;
- configure OpenCV/NumPy-friendly threading once;
- keep the helpers optional so every example still runs standalone.
"""
from __future__ import annotations

import os
import threading
from typing import Generic, Optional, TypeVar

T = TypeVar("T")


def configure_runtime_threads(cv_threads: int | None = None) -> None:
    """Set conservative OpenCV thread count to avoid CPU oversubscription.

    On RDK-class boards, multiple ROS2 nodes can otherwise each create their
    own OpenCV worker pool. The default leaves OpenCV at a small fixed pool.
    Set ZT_OPENCV_THREADS=0 to leave OpenCV untouched.
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
    """Thread-safe latest-value slot.

    A camera stream should normally drop stale frames rather than queue them.
    This is intentionally tiny and has no blocking wait semantics.
    """
    __slots__ = ("_value", "_lock")

    def __init__(self, value: Optional[T] = None):
        self._value = value
        self._lock = threading.Lock()

    def put(self, value: T) -> None:
        with self._lock:
            self._value = value

    def get(self) -> Optional[T]:
        with self._lock:
            return self._value
