#!/usr/bin/env python3
"""04 目标检测：普通 USB 摄像头 + BPU 量化 YOLO（与例程 03/05 同相机）。"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

DEMO_ROOT = '/app/zettatree_demo'
COMMON = os.path.join(DEMO_ROOT, '_common')
SCRIPT_DIR = os.path.join(DEMO_ROOT, '04_object_detection')


def generate_launch_description():
    """拉起 USB 相机与目标检测节点。"""
    camera_source = LaunchConfiguration('camera_source')
    camera_device = LaunchConfiguration('camera_device')
    show = LaunchConfiguration('show')
    score_thres = LaunchConfiguration('score_thres')
    nms_thres = LaunchConfiguration('nms_thres')

    # 相机只发话题；检测节点负责画面输出
    camera_node = ExecuteProcess(
        cmd=[
            'bash', os.path.join(COMMON, 'start_vision_cam.sh'),
            '--source', camera_source,
            '--device', camera_device,
            '--no-show',
        ],
        output='screen',
        name='vision_cam',
    )

    detection = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'object_detection.py'),
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
            '--score-thres', score_thres,
            '--nms-thres', nms_thres,
        ],
        output='screen',
        name='object_detection',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source', default_value='usb',
            description='usb=普通 USB 摄像头（本例程默认）；mipi/auto 可选 GS130W'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='USB 摄像头设备（默认 /dev/video0）'),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='推理画面输出（弹窗/快照，无显示环境自动回退）'),
        DeclareLaunchArgument(
            'score_thres', default_value='0.25',
            description='置信度阈值（概率域，官方默认 0.25）'),
        DeclareLaunchArgument(
            'nms_thres', default_value='0.45',
            description='NMS IoU 阈值（官方默认 0.45）'),
        camera_node,
        detection,
    ])
