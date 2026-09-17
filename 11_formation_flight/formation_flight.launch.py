#!/usr/bin/env python3
"""11 编队飞行：拉起 MAVROS、台架模拟、OFFBOARD 管理器与编队节点。

每架飞机都跑一套本 launch（各机处于同一 ROS Domain，`drone_id` 各不相同）。
"""
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
SCRIPT_DIR = os.path.join(DEMO_ROOT, '11_formation_flight')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    drone_id = LaunchConfiguration('drone_id')
    num_drones = LaunchConfiguration('num_drones')

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

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'formation_flight.py'),
            '--drone-id', drone_id,
            '--num-drones', num_drones,
        ],
        output='screen',
        name='formation_flight',
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
            'drone_id', default_value='0',
            description='本机编号，0 为领队（每机不同）'),
        DeclareLaunchArgument(
            'num_drones', default_value='3',
            description='编队飞机数'),
        mavros,
        manager,
        task,
        TimerAction(period=1.0, actions=[simulator]),
    ])
