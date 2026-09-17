#!/usr/bin/env python3
"""共享画面输出器：弹窗显示 / 周期快照，无显示环境自动回退。

被 _common/camera_node.py 与 04/05/06/07 的任务节点共用：
  - show=True 时弹窗显示，窗口内按 q / Esc 停止画面输出（不退出节点）；
  - 无显示环境（headless）或显式传 snapshot 路径时，按周期写 JPEG 快照；
  - 快照默认写到 fallback_path，各例程路径不同，避免互相覆盖。

任务节点接入方式（以 04 为例）：
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
    from frame_output import FrameOutput
"""
import os
import time

import cv2


class FrameOutput:
    """统一管理 OpenCV 弹窗与 JPEG 快照输出。

    无 DISPLAY/WAYLAND 时绝不调用 namedWindow（板端 Qt 后端会 abort 进程）。
    """

    def __init__(self, node, show=False, snapshot=None, snapshot_period=5.0,
                 title='frame (q/Esc 退出)', fallback_path='/tmp/frame.jpg',
                 allow_file_fallback=True):
        """node: rclpy 节点，仅用于打日志。

        allow_file_fallback=False 时：无 DISPLAY 不写 JPEG，只依赖话题视频流。
        """
        self.node = node
        self.snapshot_path = snapshot
        self.snapshot_period = max(0.1, float(snapshot_period))
        self.fallback_path = fallback_path
        self.allow_file_fallback = bool(allow_file_fallback)
        self.last_snapshot = 0.0
        self.window_ok = False
        self.title = title
        self.quit = False
        if show:
            # 必须先判断 DISPLAY：板端 OpenCV 是 Qt 后端，无显示环境时
            # namedWindow 会直接 abort 整个进程（不是 Python 异常，拦不住）
            if not (os.environ.get('DISPLAY')
                    or os.environ.get('WAYLAND_DISPLAY')):
                self._fallback('当前无显示环境（DISPLAY 未设置）')
            else:
                try:
                    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                    self.window_ok = True
                    node.get_logger().info(
                        '实时视频窗口已打开，按 q / Esc 关闭窗口')
                except Exception as e:
                    self._fallback(f'打不开窗口（{e}）')

    def _fallback(self, reason):
        """弹窗失败时的回退：优先已有 snapshot，再 fallback_path，否则只打日志。"""
        if self.snapshot_path:
            self.node.get_logger().warn(
                f'{reason}，继续写快照：{self.snapshot_path}')
            return
        if self.allow_file_fallback and self.fallback_path:
            self.snapshot_path = self.fallback_path
            self.node.get_logger().warn(
                f'{reason}，自动改为快照输出：{self.snapshot_path}')
            return
        self.node.get_logger().warn(
            f'{reason}；不写文件，请订阅视频流话题（如 /drone/depth/image）')

    def enabled(self):
        """是否需要输出画面。关闭时调用方可跳过画框/拷贝等开销。"""
        return self.window_ok or bool(self.snapshot_path)

    def output(self, frame):
        """输出一帧（先弹窗，后快照）。返回 False 表示用户按了退出键。"""
        if self.window_ok:
            try:
                cv2.imshow(self.title, frame)
                if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                    self.quit = True
                    self.node.get_logger().info('收到退出键，停止画面输出')
                    return False
            except Exception as e:
                self.window_ok = False
                self._fallback(f'显示失败（{e}）')
        if self.snapshot_path:
            now = time.monotonic()
            if now - self.last_snapshot >= self.snapshot_period:
                self.last_snapshot = now
                try:
                    cv2.imwrite(self.snapshot_path, frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 85])
                    self.node.get_logger().info(
                        f'快照已更新：{self.snapshot_path}')
                except Exception as e:
                    self.node.get_logger().warn(f'快照写入失败（{e}）')
        return True

    def close(self):
        """关闭 OpenCV 窗口（若曾成功创建）。"""
        if self.window_ok:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
