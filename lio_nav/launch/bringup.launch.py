"""
bringup.launch.py — Full LIO Pipeline launch for the Companion Computer.

Target: ROS 2 Jazzy / PX4 Autopilot

Launches:
  1. uXRCE-DDS Agent   (PX4 serial bridge)
  2. Foxglove Bridge   (WebSocket visualization — all topics)
  3. LIO Bridge Node   (/odom_corrected -> /fmu/in/vehicle_visual_odometry)

Usage:
  ros2 launch lio_nav bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

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

    # 2. Foxglove Bridge (all topics — Point-LIO publishes its own viz topics)
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
            'topic_whitelist': ['.*'],
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(is_foxglove),
    )

    # 3. LIO Bridge Node
    lio_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='lio_nav',
                executable='lio_bridge',
                name='lio_bridge',
                parameters=[{
                    'odom_topic': '/odom_corrected',
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

        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  lio_nav — LiDAR-Inertial Odometry Bringup'),
        LogInfo(msg='──────────────────────────────────────────────────────'),
        LogInfo(msg='  ▸ uXRCE-DDS Agent    (PX4 serial bridge)'),
        LogInfo(msg='  ▸ Foxglove Bridge    (WebSocket visualization)'),
        LogInfo(msg='  ▸ LIO Bridge         (Point-LIO -> PX4 EKF2)'),
        LogInfo(msg='══════════════════════════════════════════════════════'),

        uxrce_dds_agent,
        foxglove_bridge,
        lio_bridge,
    ])
