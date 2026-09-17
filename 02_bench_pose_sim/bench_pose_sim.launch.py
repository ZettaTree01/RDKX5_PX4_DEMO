#!/usr/bin/env python3
"""02 台架位姿模拟器：拉起 MAVROS、OFFBOARD 管理器与位姿模拟器。

台架场景需要「管理器发设定点 → 模拟器跟随 → 回灌 vision_pose → 飞控解锁」整条链路。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

DEMO_ROOT = '/app/zettatree_demo'
COMMON = os.path.join(DEMO_ROOT, '_common')
SCRIPT_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    max_speed = LaunchConfiguration('max_speed')
    rate = LaunchConfiguration('rate')

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
            '--bench',
        ],
        output='screen',
        name='offboard_manager',
    )

    simulator = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'bench_pose_sim.py'),
            '--max-speed', max_speed,
            '--rate', rate,
        ],
        output='screen',
        name='bench_pose_sim',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyS2:57600',
            description='X5↔飞控 40PIN UART2 串口及波特率'),
        DeclareLaunchArgument(
            'arm', default_value='false',
            description='true 才切 OFFBOARD 并解锁（台架必须拆桨）'),
        DeclareLaunchArgument(
            'altitude', default_value='0.1',
            description='起飞高度（米）；室内台架默认实飞 2 m 的 1/20'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.1',
            description='模拟器跟随限速（m/s）；室内台架默认实飞 2 的 1/20'),
        DeclareLaunchArgument(
            'rate', default_value='5.0',
            description='位姿回灌频率（Hz）；57600 UART 默认 5，与 05–08 一致'),
        mavros,
        manager,
        TimerAction(period=1.0, actions=[simulator]),
    ])
