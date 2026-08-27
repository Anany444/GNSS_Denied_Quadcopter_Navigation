"""
realsense.launch.py — Intel RealSense D455f hardware launch.

Target: Jetson Orin Nano · ROS 2 Jazzy

Launches:
  1. RealSense ROS 2 driver (realsense2_camera rs_launch.py)
     with color + depth enabled, streams aligned, sync enabled.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',

            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
        }.items()
    )

    return LaunchDescription([
        camera_fps_arg,
        camera_resolution_arg,

        LogInfo(msg='══════════════════════════════════════════════'),
        LogInfo(msg='  Launching Intel RealSense D455f (rs_launch.py)'),
        LogInfo(msg='══════════════════════════════════════════════'),

        realsense_launch,
    ])
