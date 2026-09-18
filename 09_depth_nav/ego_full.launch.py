#!/usr/bin/env python3
"""例程9：完整 C++ EGO-Planner（ZJU ego-planner-swarm ros2_version）。

链路：
  Stereonet 点云 → cloud_cam_to_world → grid_map/cloud
  MAVROS pose → pose_to_odom → /odom_world
  ego_planner_node → B 样条 → traj_server → PositionCommand
  → poscmd_to_offboard → /drone/setpoint_position/local
OpenCV：深彩 | 三维俯视（不变）；RViz：与例程8相同的官方彩色点云 + grid_map + 规划 Marker。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DEMO_ROOT = '/app/zettatree_demo'
COMMON = os.path.join(DEMO_ROOT, '_common')
SCRIPT_DIR = os.path.join(DEMO_ROOT, '09_depth_nav')
STEREO_DIR = os.path.join(DEMO_ROOT, '08_depth_camera')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
BRIDGES = os.path.join(SCRIPT_DIR, 'bridges')
INDOOR_ALT = '0.1'
INDOOR_MAX_VEL = '0.02'
ROS_LOG_DIR = '/userdata/.roslog'
TMP_LOG_DIR = '/tmp/zettatree_roslog'


def generate_launch_description():
    """完整 C++ EGO-Planner + 桥接 + Stereonet + depth_nav 安全/可视化。"""
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

    tip = LogInfo(msg=[
        '例程9：完整 C++ EGO-Planner + Stereonet。',
        ' OpenCV=深彩|3D POINT 俯视；RViz=官方彩色点云+OccViz+路径。',
        ' 需先 bash setup_full_ego.sh。arm:=false 监视；拆桨后 arm:=true。',
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

    # 室内 MAVROS local 带气压高度和残留 XY，EGO 地图只有 ±4 m。
    # z_align 把起飞位锁成 EGO 原点，cloud/poscmd 按同一基准换系。
    pose_odom = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'pose_to_odom.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='pose_to_odom',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    cloud_world = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'cloud_cam_to_world.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='cloud_cam_to_world',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    occ_viz = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'ego_occ_viz.py')],
        output='screen',
        name='ego_occ_viz',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    poscmd_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'poscmd_to_offboard.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='poscmd_to_offboard',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    # 室内小地图 + 低速；只用已滤波的 world 点云建图（原始深度近零点会标到机体上）
    ego_planner = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', '/odom_world'),
            ('grid_map/odom', '/odom_world'),
            ('grid_map/cloud', '/drone/ego/cloud_world'),
            ('planning/bspline', '/planning/bspline'),
            ('planning/data_display', '/planning/data_display'),
            ('goal_point', '/goal_point'),
            ('global_list', '/global_list'),
            ('init_list', '/init_list'),
            ('optimal_list', '/optimal_list'),
            ('a_star_list', '/a_star_list'),
            ('grid_map/occupancy', '/grid_map/occupancy'),
            ('grid_map/occupancy_inflate', '/grid_map/occupancy_inflate'),
        ],
        parameters=[{
            'fsm/flight_type': 2,
            'fsm/thresh_replan_time': 1.0,
            'fsm/thresh_no_replan_meter': 0.05,
            'fsm/planning_horizon': 2.5,
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': True,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': 4,
            'fsm/waypoint0_x': 0.05,
            'fsm/waypoint0_y': 0.0,
            'fsm/waypoint0_z': 0.1,
            'fsm/waypoint1_x': 0.05,
            'fsm/waypoint1_y': 0.05,
            'fsm/waypoint1_z': 0.1,
            'fsm/waypoint2_x': 0.0,
            'fsm/waypoint2_y': 0.05,
            'fsm/waypoint2_z': 0.1,
            'fsm/waypoint3_x': 0.0,
            'fsm/waypoint3_y': 0.0,
            'fsm/waypoint3_z': 0.1,
            'grid_map/resolution': 0.1,
            'grid_map/map_size_x': 8.0,
            'grid_map/map_size_y': 8.0,
            'grid_map/map_size_z': 2.0,
            'grid_map/local_update_range_x': 4.0,
            'grid_map/local_update_range_y': 4.0,
            'grid_map/local_update_range_z': 2.0,
            'grid_map/obstacles_inflation': 0.15,
            'grid_map/local_map_margin': 10,
            'grid_map/ground_height': -0.5,
            'grid_map/cx': 320.0,
            'grid_map/cy': 176.0,
            'grid_map/fx': 328.379,
            'grid_map/fy': 328.379,
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.35,
            'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 3,
            'grid_map/p_hit': 0.65,
            'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90,
            'grid_map/p_occ': 0.80,
            'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': 4.5,
            'grid_map/virtual_ceil_height': 1.8,
            'grid_map/visualization_truncate_height': 2.5,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 2,
            'grid_map/frame_id': 'world',
            'manager/max_vel': 0.05,
            'manager/max_acc': 0.1,
            'manager/max_jerk': 1.0,
            'manager/control_points_distance': 0.15,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': 2.5,
            'manager/use_distinctive_trajs': False,
            'manager/drone_id': 0,
            'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 0.5,
            'optimization/lambda_feasibility': 0.1,
            'optimization/lambda_fitness': 1.0,
            'optimization/dist0': 0.3,
            'optimization/swarm_clearance': 0.3,
            'optimization/max_vel': 0.05,
            'optimization/max_acc': 0.1,
            'bspline/limit_vel': 0.05,
            'bspline/limit_acc': 0.1,
            'bspline/limit_ratio': 1.1,
            'prediction/obj_num': 0,
            'prediction/lambda': 1.0,
            'prediction/predict_rate': 1.0,
        }],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[
            ('planning/bspline', '/planning/bspline'),
            ('position_cmd', '/position_cmd'),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
    )

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'depth_nav.py'),
            '--source', source,
            '--max-vel', max_vel,
            '--safe-distance', safe_distance,
            '--stop-distance', stop_distance,
            '--side', side,
            '--control-mode', 'safety',
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
        ],
        output='screen',
        name='depth_nav',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    map_world_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map',
            '--child-frame-id', 'world',
        ],
        output='log',
        name='map_world_tf',
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'bash', os.path.join(COMMON, 'rviz_run.sh'),
            os.path.join(SCRIPT_DIR, 'ego_full.rviz'),
        ],
        output='screen',
        name='rviz_ego_full',
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
            'rviz', default_value='true',
            description='默认 true：必须开 RViz（官方点云 + OccViz）。SSH 也会挂到本机桌面 :0'),
        DeclareLaunchArgument('max_vel', default_value=INDOOR_MAX_VEL),
        DeclareLaunchArgument('safe_distance', default_value='1.2'),
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('side', default_value='0.05'),
        tip,
        mavros,
        manager,
        caminfo,
        map_world_tf,
        pose_odom,
        cloud_world,
        occ_viz,
        poscmd_bridge,
        TimerAction(period=1.0, actions=[simulator]),
        TimerAction(period=1.5, actions=[stereonet]),
        TimerAction(period=4.0, actions=[ego_planner]),
        TimerAction(period=4.2, actions=[traj_server]),
        TimerAction(period=5.0, actions=[task]),
        TimerAction(period=7.5, actions=[rviz_node]),
    ])
