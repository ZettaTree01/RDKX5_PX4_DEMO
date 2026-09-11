#!/usr/bin/env python3
"""05 摄像头识别避障：拉起 MAVROS、台架模拟、OFFBOARD 管理器、相机与避障节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

DEMO_ROOT = '/app/zettatree_demo'
COMMON = os.path.join(DEMO_ROOT, '_common')
SCRIPT_DIR = os.path.join(DEMO_ROOT, '05_obstacle_avoidance')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'
INDOOR_MAX_VEL = '0.025'


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    camera_device = LaunchConfiguration('camera_device')
    show = LaunchConfiguration('show')
    hfov = LaunchConfiguration('hfov')
    safe_distance = LaunchConfiguration('safe_distance')
    max_vel = LaunchConfiguration('max_vel')
    score_thres = LaunchConfiguration('score_thres')
    nms_thres = LaunchConfiguration('nms_thres')
    infer_hz = LaunchConfiguration('infer_hz')

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'),
                         'launch', 'node.launch')),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1',
            'pluginlists_yaml': os.path.join(COMMON, 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(
                get_package_share_directory('mavros'),
                'launch', 'px4_config.yaml'),
        }.items(),
    )

    manager = ExecuteProcess(
        cmd=[
            'python3', os.path.join(COMMON, 'offboard_manager.py'),
            '--altitude', altitude,
            PythonExpression([
                "'--arm' if '", arm, "'.lower() == 'true' else '--no-arm'"]),
            PythonExpression([
                "'--bench' if '", bench, "'.lower() == 'true' else '--no-bench'"]),
        ],
        output='screen',
        name='offboard_manager',
    )

    simulator = ExecuteProcess(
        cmd=[
            'python3', os.path.join(BENCH_DIR, 'bench_pose_sim.py'),
            '--rate', '5',
        ],
        output='screen',
        name='bench_pose_sim',
        condition=IfCondition(bench),
    )

    camera_node = ExecuteProcess(
        cmd=[
            'python3', os.path.join(COMMON, 'camera_node.py'),
            '--device', camera_device,
            '--no-show',
        ],
        output='screen',
        name='camera_node',
    )

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'obstacle_avoidance.py'),
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
            '--hfov', hfov,
            '--safe-distance', safe_distance,
            '--max-vel', max_vel,
            '--score-thres', score_thres,
            '--nms-thres', nms_thres,
            '--infer-hz', infer_hz,
        ],
        output='screen',
        name='obstacle_avoidance',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyS2:57600',
            description='X5↔飞控 40PIN UART2 串口及波特率'),
        DeclareLaunchArgument(
            'arm', default_value='false',
            description='true 才切 OFFBOARD 并解锁（电机才会转）'),
        DeclareLaunchArgument(
            'altitude', default_value=INDOOR_ALT,
            description='起飞高度（米）；室内调试默认实飞 2 m 的 1/20'),
        DeclareLaunchArgument(
            'bench', default_value='true',
            description='true 拉起台架位姿模拟并写 EKF 外部视觉参数（室内无 GPS）'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='摄像头设备'),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='避障画面输出（弹窗/快照，无显示环境自动回退）'),
        DeclareLaunchArgument(
            'hfov', default_value='90.0',
            description='摄像头水平视场角（度），常见 USB 广角约 90'),
        DeclareLaunchArgument(
            'safe_distance', default_value='4.0',
            description='开始按比例后退的距离（米）；越近速度越大'),
        DeclareLaunchArgument(
            'max_vel', default_value=INDOOR_MAX_VEL,
            description='避障反向速度上限（m/s）；室内调试默认实飞 0.5 的 1/20'),
        DeclareLaunchArgument(
            'score_thres', default_value='0.25',
            description='置信度阈值（概率域，官方默认 0.25）'),
        DeclareLaunchArgument(
            'nms_thres', default_value='0.45',
            description='NMS IoU 阈值（官方默认 0.45）'),
        DeclareLaunchArgument(
            'infer_hz', default_value='10.0',
            description='YOLO 推理频率（Hz），与图像帧率解耦'),
        mavros,
        manager,
        camera_node,
        task,
        # 台架位姿 5 Hz，给 57600 UART 留出参数拉取带宽。
        TimerAction(period=1.0, actions=[simulator]),
    ])
