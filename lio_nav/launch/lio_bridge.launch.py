"""
lio_bridge.launch.py — Launch the LIO bridge node (Point-LIO -> PX4 Visual Odometry).

Target: ROS 2 Jazzy / PX4 Autopilot over uXRCE-DDS
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom_corrected',
        description='Input Point-LIO odometry topic'
    )

    px4_visual_odom_arg = DeclareLaunchArgument(
        'px4_visual_odom_topic',
        default_value='/fmu/in/vehicle_visual_odometry',
        description='Output PX4 VehicleOdometry topic'
    )

    lio_bridge_node = Node(
        package='lio_nav',
        executable='lio_bridge',
        name='lio_bridge',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
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
        odom_topic_arg,
        px4_visual_odom_arg,

        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  Launching Point-LIO -> PX4 LIO Bridge Node'),
        LogInfo(msg='  Initial pose zeroing (OdomCorrector) enabled'),
        LogInfo(msg='══════════════════════════════════════════════════════'),

        lio_bridge_node,
    ])
