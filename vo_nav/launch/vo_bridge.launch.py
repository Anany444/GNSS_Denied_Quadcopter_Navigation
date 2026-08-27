"""
vo_bridge.launch.py — Launch the VO bridge node (RTAB-Map -> PX4 Visual Odometry).

Target: ROS 2 Jazzy / PX4 Autopilot over uXRCE-DDS
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    rtabmap_odom_arg = DeclareLaunchArgument(
        'rtabmap_odom_topic',
        default_value='/rtabmap/odom',
        description='Input RTAB-Map odometry topic'
    )

    px4_visual_odom_arg = DeclareLaunchArgument(
        'px4_visual_odom_topic',
        default_value='/fmu/in/vehicle_visual_odometry',
        description='Output PX4 VehicleOdometry topic'
    )

    vo_bridge_node = Node(
        package='vo_nav',
        executable='vo_bridge',
        name='vo_bridge',
        parameters=[{
            'rtabmap_odom_topic': LaunchConfiguration('rtabmap_odom_topic'),
            'px4_visual_odom_topic': LaunchConfiguration('px4_visual_odom_topic'),
            'diag_publish_interval_s': 5.0,
            'max_position_jump_m': 2.0,
            'fallback_position_std': 0.05,
            'fallback_orientation_std': 0.01,
            'fallback_velocity_std': 0.05,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        rtabmap_odom_arg,
        px4_visual_odom_arg,

        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  Launching RTAB-Map -> PX4 VO Bridge Node'),
        LogInfo(msg='  Converting ENU -> NED & FLU -> FRD for EKF2'),
        LogInfo(msg='══════════════════════════════════════════════════════'),

        vo_bridge_node,
    ])
