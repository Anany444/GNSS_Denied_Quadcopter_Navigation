"""
bringup.launch.py — Full LIO Pipeline launch for the Companion Computer.

Target: ROS 2 Jazzy / PX4 Autopilot

Launches:
  1. uXRCE-DDS Agent      (PX4 serial bridge)
  2. Foxglove Bridge      (WebSocket visualization — all topics)
  3. Unitree L2 Driver    (LiDAR point cloud + IMU over Ethernet)
  4. Point-LIO            (LiDAR-Inertial odometry + 3D mapping)
  5. LIO Bridge Node      (/odom_corrected -> /fmu/in/vehicle_visual_odometry)

Usage:
  ros2 launch lio_nav bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    # 3. Unitree L2 LiDAR Driver (Ethernet — publishes /unilidar/cloud and /unilidar/imu)
    unitree_lidar_driver = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},
            {'work_mode': 0},
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 30},
            # Ethernet connection
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.1.2'},
            # Topic / frame names expected by Point-LIO unilidar_l2 config
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    # 4. Point-LIO — LiDAR-Inertial Odometry + 3D Mapping
    #    Includes the upstream mapping_unilidar_l2 launch, with rviz disabled
    #    for headless Jetson operation.
    point_lio_launch = TimerAction(
        period=3.0,   # wait for LiDAR driver to come up and send first scans
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('point_lio'),
                        'launch',
                        'mapping_unilidar_l2.launch.py'
                    ])
                ),
                launch_arguments={
                    'rviz': 'false',
                }.items()
            ),
        ],
    )

    # 5. LIO Bridge Node (waits for Point-LIO to initialise and publish /odom_corrected)
    lio_bridge = TimerAction(
        period=6.0,
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
        LogInfo(msg='  ▸ Unitree L2 Driver  (LiDAR + IMU over Ethernet)'),
        LogInfo(msg='  ▸ Point-LIO          (LiDAR-Inertial odometry + mapping)'),
        LogInfo(msg='  ▸ LIO Bridge         (Point-LIO -> PX4 EKF2)'),
        LogInfo(msg='══════════════════════════════════════════════════════'),

        uxrce_dds_agent,
        foxglove_bridge,
        unitree_lidar_driver,
        point_lio_launch,
        lio_bridge,
    ])
