#!/usr/bin/env python3
"""10 目标跟随：MAVROS + Stereonet + YOLO 行人 + EGO/直接位置跟随。

planner:=ego 时拉起例程 9 的完整 C++ EGO（flight_type=MANUAL_TARGET），
目标点发布到 /move_base_simple/goal（与 Fast-Planner / EGO 一致）。
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
SCRIPT_DIR = os.path.join(DEMO_ROOT, '10_target_follow')
NAV_DIR = os.path.join(DEMO_ROOT, '09_depth_nav')
BRIDGES = os.path.join(NAV_DIR, 'bridges')
STEREO_DIR = os.path.join(DEMO_ROOT, '08_depth_camera')
BENCH_DIR = os.path.join(DEMO_ROOT, '02_bench_pose_sim')
INDOOR_ALT = '0.1'
INDOOR_MAX_VEL = '0.02'
ROS_LOG_DIR = '/userdata/.roslog'
TMP_LOG_DIR = '/tmp/zettatree_roslog'


def generate_launch_description():
    """例程10：在深度导航栈上叠加行人跟随。"""
    fcu_url = LaunchConfiguration('fcu_url')
    arm = LaunchConfiguration('arm')
    altitude = LaunchConfiguration('altitude')
    bench = LaunchConfiguration('bench')
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    planner = LaunchConfiguration('planner')
    start_stereo = LaunchConfiguration('start_stereo')
    rviz = LaunchConfiguration('rviz')
    standoff = LaunchConfiguration('standoff')
    follow_z = LaunchConfiguration('follow_z')
    max_vel = LaunchConfiguration('max_vel')
    safe_distance = LaunchConfiguration('safe_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    min_score = LaunchConfiguration('min_score')
    snapshot = LaunchConfiguration('snapshot')

    tip = LogInfo(msg=[
        '例程10：目标跟随（YOLO 行人 + 深度 3D + EGO/直接）。',
        ' RViz=例程8同款彩色点云(camera_link)+规划路线+跟随连线；需拆桨。',
    ])

    use_ego = PythonExpression(["'", planner, "'.lower() == 'ego'"])
    use_stereo = PythonExpression([
        "'", source, "'.lower() == 'stereonet' and '",
        start_stereo, "'.lower() == 'true'",
    ])

    # 飞控链路
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'),
                         'launch', 'node.launch')),
        launch_arguments={
            'fcu_url': fcu_url,
            # 本地 GCS 桥：gcs_heartbeat.py 往 14550 打 HEARTBEAT，消「No GCS datalink」
            'gcs_url': 'udp://0.0.0.0:14550@',
            'tgt_system': '1',
            'tgt_component': '1',
            'pluginlists_yaml': os.path.join(COMMON, 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(
                get_package_share_directory('mavros'),
                'launch', 'px4_config.yaml'),
        }.items(),
    )

    gcs_hb = ExecuteProcess(
        cmd=['python3', os.path.join(COMMON, 'gcs_heartbeat.py'),
             '--host', '127.0.0.1', '--port', '14550', '--rate', '1.0'],
        output='screen',
        name='gcs_heartbeat',
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
        cmd=['python3', os.path.join(BENCH_DIR, 'bench_pose_sim.py'),
             '--rate', '5'],
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
            '--baseline', '0.07917', '--rate', '15.0',
        ],
        output='screen',
        name='stereo_caminfo_pub',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(use_stereo),
    )

    stereonet = ExecuteProcess(
        cmd=['bash', os.path.join(STEREO_DIR, 'start_stereonet.sh'),
             'render_perf:=True',
             ],
        output='screen',
        name='hobot_stereonet',
        additional_env={'ROS_LOG_DIR': ROS_LOG_DIR},
        condition=IfCondition(use_stereo),
    )

    # z_align（例程10）：室内 MAVROS local z 是气压绝对高度（几十米），
    # EGO 地图 z 固定 [-0.5,1.5]；桥接按 /drone/ego/z_ref 统一换系
    pose_odom = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'pose_to_odom.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='pose_to_odom',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(use_ego),
    )

    cloud_world = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'cloud_cam_to_world.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='cloud_cam_to_world',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(use_ego),
    )

    occ_viz = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'ego_occ_viz.py')],
        output='screen',
        name='ego_occ_viz',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(use_ego),
    )

    poscmd_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(BRIDGES, 'poscmd_to_offboard.py'),
             '--ros-args', '-p', 'z_align:=true'],
        output='screen',
        name='poscmd_to_offboard',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(use_ego),
    )

    # MANUAL_TARGET=1：订阅 /move_base_simple/goal
    ego_planner = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', '/odom_world'),
            ('grid_map/odom', '/odom_world'),
            ('grid_map/cloud', '/drone/ego/cloud_world'),
            # 不用原始深度建图：未滤波的近零深度会投影到机体格子。
            # 点云已由 cloud_cam_to_world 滤近距并变到 EGO 世界系。
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
        # 建图参数与例程 9 一致；fsm=MANUAL_TARGET 动态目标（跟行人）
        parameters=[{
            'fsm/flight_type': 1,
            'fsm/thresh_replan_time': 0.8,
            'fsm/thresh_no_replan_meter': 0.08,
            'fsm/planning_horizon': 2.5,
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': True,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': 0,
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
        condition=IfCondition(use_ego),
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
        condition=IfCondition(use_ego),
    )

    task = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'target_follow.py'),
            '--source', source,
            '--planner', planner,
            '--standoff', standoff,
            '--follow-z', follow_z,
            '--max-vel', max_vel,
            '--safe-distance', safe_distance,
            '--stop-distance', stop_distance,
            '--min-score', min_score,
            '--snapshot', snapshot,
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
        ],
        output='screen',
        name='target_follow',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    map_world_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'world',
        ],
        output='log',
        name='map_world_tf',
        condition=IfCondition(use_ego),
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'bash', os.path.join(COMMON, 'rviz_run.sh'),
            os.path.join(SCRIPT_DIR, 'target_follow.rviz'),
        ],
        output='screen',
        name='rviz_target_follow',
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
        DeclareLaunchArgument('source', default_value='stereonet'),
        DeclareLaunchArgument('start_stereo', default_value='true'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument(
            'planner', default_value='ego',
            description='ego=完整EGO(/move_base_simple/goal)；direct=位置直跟'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='默认 true：必须开 RViz（例程8点云 + 规划/跟随）。SSH 也会挂到本机桌面 :0'),
        DeclareLaunchArgument('standoff', default_value='0.8'),
        DeclareLaunchArgument('follow_z', default_value=INDOOR_ALT),
        DeclareLaunchArgument('max_vel', default_value=INDOOR_MAX_VEL),
        DeclareLaunchArgument('safe_distance', default_value='1.2'),
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('min_score', default_value='0.25'),
        DeclareLaunchArgument(
            'snapshot', default_value='',
            description='无显示器时的 JPEG 快照路径'),
        tip,
        mavros,
        gcs_hb,
        manager,
        caminfo,
        map_world_tf,
        pose_odom,
        cloud_world,
        occ_viz,
        poscmd_bridge,
        TimerAction(period=1.0, actions=[simulator]),
        # Stereonet 与 YOLO/EGO 抢 BPU 易在启动瞬间把 inference 队列打满并 segfault
        TimerAction(period=12.0, actions=[stereonet]),
        TimerAction(period=16.0, actions=[ego_planner]),
        TimerAction(period=16.2, actions=[traj_server]),
        TimerAction(period=18.0, actions=[task]),
        TimerAction(period=20.0, actions=[rviz_node]),
    ])
