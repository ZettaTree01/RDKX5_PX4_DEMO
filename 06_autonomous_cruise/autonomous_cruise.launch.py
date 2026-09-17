#!/usr/bin/env python3
"""06 自主巡航拍摄：拉起 MAVROS、台架模拟、OFFBOARD 管理器与巡航节点。"""
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
SCRIPT_DIR = os.path.join(DEMO_ROOT, '06_autonomous_cruise')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'


def generate_launch_description():
    """组装 MAVROS、OFFBOARD 管理器、台架模拟、相机与巡航任务节点。"""
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    device = LaunchConfiguration('device')
    camera_source = LaunchConfiguration('camera_source')
    show = LaunchConfiguration('show')

    # 飞控链路：UART 接 PX4，插件列表用演示仓共用配置
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

    # OFFBOARD 管理器：接管设定点、解锁与降落；arm/bench 由 launch 参数决定
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

    # 室内无 GPS：台架位姿模拟向 EKF 喂外部视觉（bench:=true 时拉起）
    simulator = ExecuteProcess(
        cmd=[
            'python3', os.path.join(BENCH_DIR, 'bench_pose_sim.py'),
            '--rate', '5',
        ],
        output='screen',
        name='bench_pose_sim',
        condition=IfCondition(bench),
    )

    # 视觉相机：默认 auto（MIPI 优先），供巡航节点 YOLO 与画面预览
    vision_cam = ExecuteProcess(
        cmd=[
            'bash', os.path.join(COMMON, 'start_vision_cam.sh'),
            '--source', camera_source,
            '--device', device,
            '--no-show',
        ],
        output='screen',
        name='vision_cam',
    )

    # 巡航任务节点：规划方形航点并到点拍照
    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'autonomous_cruise.py'),
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
        ],
        output='screen',
        name='autonomous_cruise',
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
            description='auto=GS130W MIPI 优先（BPU YOLO 前置）；usb=USB 摄像头'),
        DeclareLaunchArgument(
            'device', default_value='/dev/video0',
            description='USB 回退设备；auto 时仍先尝试 MIPI'),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='巡航画面输出（弹窗/快照，无显示环境自动回退）'),
        mavros,
        manager,
        vision_cam,
        task,
        TimerAction(period=1.0, actions=[simulator]),
    ])
