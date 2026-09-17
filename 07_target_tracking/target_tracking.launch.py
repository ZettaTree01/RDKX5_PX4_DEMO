#!/usr/bin/env python3
"""07 停机坪 H 标对准降落：拉起 MAVROS、台架模拟、OFFBOARD 管理器、相机与降落节点。"""
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
SCRIPT_DIR = os.path.join(DEMO_ROOT, '07_target_tracking')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    camera_source = LaunchConfiguration('camera_source')
    camera_device = LaunchConfiguration('camera_device')
    show = LaunchConfiguration('show')
    max_vel = LaunchConfiguration('max_vel')

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
            'bash', os.path.join(COMMON, 'start_vision_cam.sh'),
            '--source', camera_source,
            '--device', camera_device,
            '--no-show',
        ],
        output='screen',
        name='vision_cam',
    )

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'target_tracking.py'),
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
            '--max-vel', max_vel,
        ],
        output='screen',
        name='helipad_landing',
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
            description='起飞高度（米）；室内台架默认实飞 2 m 的 1/20'),
        DeclareLaunchArgument(
            'bench', default_value='true',
            description='true 拉起台架位姿模拟并写 EKF 外部视觉参数（室内无 GPS）'),
        DeclareLaunchArgument(
            'camera_source', default_value='auto',
            description='auto=GS130W MIPI 优先；usb=USB 摄像头'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='USB 回退设备'),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='对准画面输出（弹窗/快照，无显示环境自动回退）'),
        DeclareLaunchArgument(
            'max_vel', default_value='0.05',
            description='对准平移速度上限（m/s）；室内默认实飞 1.0 的 1/20'),
        mavros,
        manager,
        camera_node,
        task,
        TimerAction(period=1.0, actions=[simulator]),
    ])
