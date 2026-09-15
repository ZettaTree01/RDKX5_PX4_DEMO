#!/usr/bin/env python3
"""08 GS130W：双目 → Depth → 官方 PointCloud2 → RViz / OpenCV 三维地图。

官方参考：hobot_mipi_cam + hobot_stereonet；RViz 订 /StereoNetNode/stereonet_pointcloud2。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

DEMO_ROOT = '/app/zettatree_demo'
SCRIPT_DIR = os.path.join(DEMO_ROOT, '08_depth_camera')
ROS_LOG_DIR = '/userdata/.roslog'
TMP_LOG_DIR = '/tmp/zettatree_roslog'


def generate_launch_description():
    for d in (TMP_LOG_DIR,):
        try:
            os.makedirs(d, exist_ok=True)
        except OSError:
            pass

    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    stride = LaunchConfiguration('stride')
    max_range = LaunchConfiguration('max_range')
    min_range = LaunchConfiguration('min_range')
    depth_topic = LaunchConfiguration('depth_topic')
    color_topic = LaunchConfiguration('color_topic')
    combine_topic = LaunchConfiguration('combine_topic')
    baseline_m = LaunchConfiguration('baseline_m')
    stereo_layout = LaunchConfiguration('stereo_layout')
    rotate_cw = LaunchConfiguration('rotate_cw')
    start_mipi = LaunchConfiguration('start_mipi')
    start_stereonet = LaunchConfiguration('start_stereonet')
    panel_mode = LaunchConfiguration('panel_mode')
    stereo_matcher = LaunchConfiguration('stereo_matcher')
    stereo_max_width = LaunchConfiguration('stereo_max_width')
    stereo_period = LaunchConfiguration('stereo_period')
    snapshot = LaunchConfiguration('snapshot')
    rviz = LaunchConfiguration('rviz')
    map_enable = LaunchConfiguration('map')
    voxel_size = LaunchConfiguration('voxel_size')
    map_accumulate = LaunchConfiguration('map_accumulate')

    env_log = SetEnvironmentVariable(name='ROS_LOG_DIR', value=ROS_LOG_DIR)

    fix_logdir = ExecuteProcess(
        cmd=['bash', '-lc',
             'mkdir -p /tmp/zettatree_roslog; '
             'if [ ! -w /userdata/.roslog ]; then '
             'echo sunrise | sudo -S mkdir -p /userdata/.roslog; '
             'echo sunrise | sudo -S chmod 777 /userdata/.roslog; fi'],
        output='screen',
        name='fix_roslog_dir',
    )

    # ① 双目摄像头 GS130W
    mipi = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mipi_cam', 'mipi_cam', '--ros-args',
            '-p', 'device_mode:=dual',
            '-p', 'out_format:=nv12',
            '-p', 'dual_combine:=2',
            '-p', 'image_width:=640',
            '-p', 'image_height:=352',
            '-p', 'framerate:=15.0',
            '-p', 'channel:=2',
            '-p', 'channel2:=0',
            '-p', 'lpwm_enable:=True',
            '-p', 'rotation:=90.0',
            '-p', 'gdc_enable:=True',
            '-p', 'frame_id:=camera_link',
            '-p', ('camera_calibration_file_path:='
                   '/opt/tros/humble/lib/mipi_cam/config/'
                   'SC132gs_dual_calibration.yaml'),
            '--log-level', 'warn',
        ],
        output='screen',
        name='mipi_cam_dual',
        additional_env={'ROS_LOG_DIR': ROS_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "('", source, "'.lower() in ('mipi_stereo', 'stereonet')) and '",
            start_mipi, "'.lower() == 'true'",
        ])),
    )

    # ② Depth（BPU Stereonet）
    stereonet = ExecuteProcess(
        cmd=['bash', os.path.join(SCRIPT_DIR, 'start_stereonet.sh')],
        output='screen',
        name='hobot_stereonet',
        additional_env={'ROS_LOG_DIR': ROS_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", source, "'.lower() == 'stereonet' and '",
            start_stereonet, "'.lower() == 'true'",
        ])),
    )

    caminfo = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'pub_stereo_caminfo.py'),
            '--width', '640', '--height', '352',
            '--fx', '328.379', '--fy', '328.379',
            '--cx', '320.0', '--cy', '176.0',
            '--baseline', baseline_m,
            '--rate', '15.0',
        ],
        output='screen',
        name='stereo_caminfo_pub',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", source, "'.lower() == 'stereonet' and '",
            start_stereonet, "'.lower() == 'true'",
        ])),
    )

    # ③ PointCloud2 转发 + OpenCV 深彩
    node = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'depth_pointcloud.py'),
            '--source', source,
            '--stride', stride,
            '--max-range', max_range,
            '--min-range', min_range,
            '--depth-topic', depth_topic,
            '--color-topic', color_topic,
            '--combine-topic', combine_topic,
            '--baseline-m', baseline_m,
            '--stereo-layout', stereo_layout,
            '--rotate-cw', rotate_cw,
            '--panel-mode', panel_mode,
            '--stereo-matcher', stereo_matcher,
            '--stereo-max-width', stereo_max_width,
            '--stereo-period', stereo_period,
            '--snapshot', snapshot,
            '--snapshot-period', '2.0',
            PythonExpression([
                "'--show' if '", show, "'.lower() == 'true' else '--no-show'"]),
        ],
        output='screen',
        name='depth_pointcloud',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
    )

    # ④a 三维地图：体素(+累积)，对齐官方 voxel 思路
    map_node = ExecuteProcess(
        cmd=[
            'python3', os.path.join(SCRIPT_DIR, 'pointcloud_map.py'),
            '--in-topic', '/drone/depth/points',
            '--out-topic', '/drone/map/points',
            '--voxel-size', voxel_size,
            '--max-points', '12000',
            '--pub-hz', '2.0',
            '--max-range', max_range,
            '--min-range', min_range,
            '--frame-id', 'camera_link',
            PythonExpression([
                "'--accumulate' if '", map_accumulate,
                "'.lower() == 'true' else '--no-accumulate'"]),
        ],
        output='screen',
        name='pointcloud_map',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", map_enable, "'.lower() == 'true'",
        ])),
    )

    tip = LogInfo(msg=[
        '例程8：OpenCV【深彩|三维俯视】+ RViz【官方彩色点云 '
        '/StereoNetNode/stereonet_pointcloud2，Fixed Frame=camera_link】。',
    ])

    # 可选 RViz：直接订官方 XYZRGB，无需 map→camera_link TF
    rviz_node = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then '
            'echo "[rviz] 无 DISPLAY，跳过"; exit 0; fi; '
            f'exec rviz2 -d {SCRIPT_DIR}/depth_cloud.rviz',
        ],
        output='screen',
        name='rviz_depth_cloud',
        additional_env={'ROS_LOG_DIR': TMP_LOG_DIR},
        condition=IfCondition(PythonExpression([
            "'", rviz, "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='stereonet',
            description='stereonet(BPU)|mipi_stereo(CPU)|simulate|orbbec|realsense'),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='OpenCV 深彩（Depth 可视化）'),
        DeclareLaunchArgument('stride', default_value='4'),
        DeclareLaunchArgument('max_range', default_value='5.0'),
        DeclareLaunchArgument('min_range', default_value='0.3'),
        DeclareLaunchArgument('depth_topic', default_value='__default__'),
        DeclareLaunchArgument('color_topic', default_value='__default__'),
        DeclareLaunchArgument('combine_topic', default_value='__default__'),
        DeclareLaunchArgument(
            'baseline_m', default_value='0.07917',
            description='GS130W 基线约 79.2 mm'),
        DeclareLaunchArgument('stereo_layout', default_value='tb'),
        DeclareLaunchArgument(
            'rotate_cw', default_value='0',
            description='仅 mipi_stereo'),
        DeclareLaunchArgument(
            'panel_mode', default_value='depth',
            description='depth|depth_cloud|cloud|full'),
        DeclareLaunchArgument('stereo_matcher', default_value='bm'),
        DeclareLaunchArgument('stereo_max_width', default_value='320'),
        DeclareLaunchArgument('stereo_period', default_value='1.0'),
        DeclareLaunchArgument('snapshot', default_value='none'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='默认 true：RViz 显示官方彩色点云 /StereoNetNode/stereonet_pointcloud2'),
        DeclareLaunchArgument(
            'map', default_value='false',
            description='独立 pointcloud_map 节点；默认 false（已由 depth_pointcloud 内建）'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='仅 map:=true 时用于独立地图节点'),
        DeclareLaunchArgument(
            'map_accumulate', default_value='true',
            description='仅 map:=true'),
        DeclareLaunchArgument(
            'start_mipi', default_value='false',
            description='默认 false：由 ensure_mipi_bpu.sh 拉起'),
        DeclareLaunchArgument(
            'start_stereonet', default_value='true',
            description='拉起 BPU Stereonet'),
        fix_logdir,
        env_log,
        tip,
        mipi,
        caminfo,
        TimerAction(period=1.5, actions=[stereonet]),
        TimerAction(period=5.0, actions=[node]),
        TimerAction(period=6.0, actions=[map_node]),
        TimerAction(period=7.5, actions=[rviz_node]),
    ])
