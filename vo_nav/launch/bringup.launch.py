"""
bringup.launch.py — Full VO Pipeline launch for the Companion Computer.

Target: ROS 2 Jazzy / PX4 Autopilot

Launches:
  1. uXRCE-DDS Agent        (PX4 serial bridge)
  2. Foxglove Bridge         (WebSocket visualization)
  3. RealSense D455f Driver  (RGB-D camera)
  4. RTAB-Map                (RGB-D odometry + SLAM)
  5. Foxglove Relay Node     (compressed image streams over WiFi)
  6. VO Bridge Node          (/rtabmap/odom -> /fmu/in/vehicle_visual_odometry)

Usage:
  ros2 launch vo_nav bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('vo_nav')

    # --- uXRCE-DDS Agent ---
    dds_serial_port_arg = DeclareLaunchArgument(
        'dds_serial_port',
        default_value='/dev/ttyCH341USB0',
        description='Serial device connected to PX4'
    )

    dds_serial_baud_arg = DeclareLaunchArgument(
        'dds_serial_baud',
        default_value='921600',
        description='Serial baudrate'
    )

    # --- Foxglove Bridge ---
    enable_foxglove_arg = DeclareLaunchArgument(
        'enable_foxglove',
        default_value='true',
        description='Launch Foxglove WebSocket bridge'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='WebSocket port for Foxglove Studio'
    )

    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='40',
        description='JPEG quality for Foxglove color stream'
    )

    foxglove_max_fps_arg = DeclareLaunchArgument(
        'foxglove_max_fps',
        default_value='15.0',
        description='Max FPS for Foxglove compressed streams'
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera framerate'
    )

    camera_resolution_arg = DeclareLaunchArgument(
        'camera_resolution',
        default_value='640x480',
        description='Camera resolution WxH'
    )

    is_foxglove = PythonExpression(["'", LaunchConfiguration('enable_foxglove'), "' == 'true'"])

    # 1. Micro XRCE-DDS Agent
    uxrce_dds_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'serial',
            '--dev', LaunchConfiguration('dds_serial_port'),
            '-b', LaunchConfiguration('dds_serial_baud'),
        ],
        name='uxrce_dds_agent',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
    )

    # 2. Foxglove Bridge (whitelisted topics for VO pipeline)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'send_buffer_limit': 10000000,
            'num_threads': 2,
            'min_qos_depth': 1,
            'max_qos_depth': 1,
            'use_compression': True,
            'topic_whitelist': [
                '/foxglove/.*',
                '/rtabmap/.*',
                '/fmu/.*',
                '/tf.*',
                '/camera/camera/.*camera_info',
                '/vo_nav/diagnostics',
            ],
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(is_foxglove),
    )

    # 3. RealSense D455f Driver
    realsense_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'realsense.launch.py')
                ),
                launch_arguments={
                    'camera_fps': LaunchConfiguration('camera_fps'),
                    'camera_resolution': LaunchConfiguration('camera_resolution'),
                }.items(),
            ),
        ],
    )

    # 4. RTAB-Map RGB-D Odometry + SLAM
    rtabmap_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('rtabmap_launch'),
                        'launch',
                        'rtabmap.launch.py'
                    )
                ),
                launch_arguments={
                    'rgbd': 'true',
                    'rgb_topic': '/camera/camera/color/image_raw',
                    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                    'camera_info_topic': '/camera/camera/color/camera_info',
                    'frame_id': 'camera_link',
                    'visual_odometry': 'true',
                    'approx_sync': 'false',
                    'rviz': 'false',
                }.items()
            ),
        ],
    )

    # 5. Foxglove Relay Node
    foxglove_relay = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='vo_nav',
                executable='foxglove_relay',
                name='foxglove_relay',
                parameters=[{
                    'color_topic_in': '/camera/camera/color/image_raw',
                    'depth_topic_in': '/camera/camera/aligned_depth_to_color/image_raw',
                    'infra1_topic_in': '/camera/camera/infra1/image_rect_raw',
                    'infra2_topic_in': '/camera/camera/infra2/image_rect_raw',
                    'color_topic_out': '/foxglove/color/compressed',
                    'depth_topic_out': '/foxglove/depth/compressed',
                    'infra1_topic_out': '/foxglove/infra1/compressed',
                    'infra2_topic_out': '/foxglove/infra2/compressed',
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                    'max_fps': LaunchConfiguration('foxglove_max_fps'),
                    'downscale_factor': 1.0,
                    'enable_depth': True,
                    'enable_infra': True,
                }],
                output='screen',
                emulate_tty=True,
            ),
        ],
        condition=IfCondition(is_foxglove),
    )

    # 6. VO Bridge Node
    vo_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='vo_nav',
                executable='vo_bridge',
                name='vo_bridge',
                parameters=[{
                    'rtabmap_odom_topic': '/rtabmap/odom',
                    'px4_visual_odom_topic': '/fmu/in/vehicle_visual_odometry',
                    'diag_publish_interval_s': 5.0,
                    'max_position_jump_m': 2.0,
                    'fallback_position_std': 0.05,
                    'fallback_orientation_std': 0.01,
                    'fallback_velocity_std': 0.05,
                }],
                output='screen',
                emulate_tty=True,
            ),
        ],
    )

    return LaunchDescription([
        dds_serial_port_arg,
        dds_serial_baud_arg,
        enable_foxglove_arg,
        foxglove_port_arg,
        jpeg_quality_arg,
        foxglove_max_fps_arg,
        camera_fps_arg,
        camera_resolution_arg,

        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  vo_nav — RGB-D Visual Odometry Bringup'),
        LogInfo(msg='──────────────────────────────────────────────────────'),
        LogInfo(msg='  ▸ uXRCE-DDS Agent    (PX4 serial bridge)'),
        LogInfo(msg='  ▸ Foxglove Bridge    (WebSocket visualization)'),
        LogInfo(msg='  ▸ RealSense D455f    (RGB-D camera driver)'),
        LogInfo(msg='  ▸ RTAB-Map           (RGB-D odometry + SLAM)'),
        LogInfo(msg='  ▸ Foxglove Relay     (compressed image streams)'),
        LogInfo(msg='  ▸ VO Bridge          (RTAB-Map -> PX4 EKF2)'),
        LogInfo(msg='══════════════════════════════════════════════════════'),

        uxrce_dds_agent,
        foxglove_bridge,
        realsense_launch,
        rtabmap_launch,
        foxglove_relay,
        vo_bridge,
    ])
