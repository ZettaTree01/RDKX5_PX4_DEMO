#!/usr/bin/env python3
"""09 深度导航：MAVROS + Stereonet + EGO 局部地图/路径 + OpenCV。

OpenCV：官方深彩 | 三维俯视（不变）。
RViz2：官方彩色点云 + EGO 占据地图 + A*/最优路径。
参考：https://github.com/Kinang2/Ego-Planner-System
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

DEMO_ROOT = '/app/zettatree_demo'
COMMON = os.path.join(DEMO_ROOT, '_common')
SCRIPT_DIR = os.path.join(DEMO_ROOT, '09_depth_nav')
STEREO_DIR = os.path.join(DEMO_ROOT, '08_depth_camera')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'
INDOOR_MAX_VEL = '0.02'
ROS_LOG_DIR = '/userdata/.roslog'
TMP_LOG_DIR = '/tmp/zettatree_roslog'


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    max_vel = LaunchConfiguration('max_vel')
    safe_distance = LaunchConfiguration('safe_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    side = LaunchConfiguration('side')
    start_stereo = LaunchConfiguration('start_stereo')
    rviz = LaunchConfiguration('rviz')
    ego_enable = LaunchConfiguration('ego')

    tip = LogInfo(msg=[
        '例程9（Python A* 对照）：局部地图+路径 + Stereonet。',
        ' 完整 C++ EGO 请用 run.sh（默认）或 run_ego_full.sh。',
        ' OpenCV=深彩|三维；RViz=点云地图+规划路径。arm:=false 监视。',
    ])

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

    caminfo = ExecuteProcess(
        cmd=[
            'python3', os.path.join(STEREO_DIR, 'pub_stereo_caminfo.py'),
            '--width', '640', '--height', '352',
            '--fx', '328.379', '--fy', '328.379',
            '--cx', '320.0', '--cy', '176.0',
            '--baseline', '0.07917',
            '--rate', '15.0',
        ],
        output='screen',
        name='stereo_caminfo_pub',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", source, "'.lower() == 'stereonet' and '",
            start_stereo, "'.lower() == 'true'",
        ])),
    )

    stereonet = ExecuteProcess(
        cmd=[
            'bash', os.path.join(STEREO_DIR, 'start_stereonet.sh'),
            'pointcloud_downsample_step:=4',
            'render_perf:=True',
        ],
        output='screen',
        name='hobot_stereonet',
        additional_env={'ROS_LOG_DIR': ROS_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", source, "'.lower() == 'stereonet' and '",
            start_stereo, "'.lower() == 'true'",
        ])),
    )

    ego_node = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'ego_planner_node.py'),
            '--cloud-topic', '/StereoNetNode/stereonet_pointcloud2',
            '--max-vel', max_vel,
            '--resolution', '0.15',
            '--inflation', '0.25',
        ],
        output='screen',
        name='ego_planner_bridge',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", ego_enable, "'.lower() == 'true'",
        ])),
    )

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'depth_nav.py'),
            '--source', source,
            '--max-vel', max_vel,
            '--safe-distance', safe_distance,
            '--stop-distance', stop_distance,
            '--side', side,
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
        ],
        output='screen',
        name='depth_nav',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    static_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map',
            '--child-frame-id', 'camera_link',
        ],
        output='log',
        name='nav_cloud_tf',
        condition=IfCondition(PythonExpression([
            "'", rviz, "'.lower() == 'true'",
        ])),
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then '
            'echo "[rviz] 无 DISPLAY，跳过"; exit 0; fi; '
            f'exec rviz2 -d {SCRIPT_DIR}/depth_nav.rviz',
        ],
        output='screen',
        name='rviz_depth_nav',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", rviz, "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyS2:57600'),
        DeclareLaunchArgument('arm', default_value='false'),
        DeclareLaunchArgument('altitude', default_value=INDOOR_ALT),
        DeclareLaunchArgument('bench', default_value='true'),
        DeclareLaunchArgument(
            'source', default_value='stereonet',
            description='stereonet(GS130W/BPU)|simulate|orbbec|realsense'),
        DeclareLaunchArgument(
            'start_stereo', default_value='true',
            description='source=stereonet 时拉起例程8 Stereonet'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument(
            'ego', default_value='true',
            description='启动 EGO 局部占据地图 + A* 路径节点'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='RViz2：彩色点云 + 占据地图 + 规划路径'),
        DeclareLaunchArgument('max_vel', default_value=INDOOR_MAX_VEL),
        DeclareLaunchArgument('safe_distance', default_value='1.2'),
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('side', default_value='0.05'),
        tip,
        mavros,
        manager,
        caminfo,
        static_tf,
        TimerAction(period=1.0, actions=[simulator]),
        TimerAction(period=1.5, actions=[stereonet]),
        TimerAction(period=4.0, actions=[ego_node]),
        TimerAction(period=5.0, actions=[task]),
        TimerAction(period=7.5, actions=[rviz_node]),
    ])
