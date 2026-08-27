"""
vo_bridge_node — Bridge RTAB-Map Visual Odometry to PX4 EKF2

Target: Jetson Orin Nano running ROS 2 Jazzy

Translates RTAB-Map's visual odometry (/rtabmap/odom) into PX4's
VehicleOdometry topic (/fmu/in/vehicle_visual_odometry) over uXRCE-DDS.

Performs:
  1. Coordinate frame conversion:
     - World Position: ENU (East-North-Up) -> NED (North-East-Down)
     - Orientation:    ENU/FLU -> NED/FRD (Hamiltonian w, x, y, z)
     - Body Velocity:  FLU (Front-Left-Up) -> FRD (Front-Right-Down)
  2. Covariance mapping from 6x6 pose/twist matrices to 3D variances
  3. Microsecond timestamp mapping for uXRCE-DDS
  4. Odometry reset counter & tracking validity diagnostics
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Constant rotation matrices for frame transformations
# R_NED_ENU: Transform vector/matrix in ENU world frame to NED world frame
# x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
R_NED_ENU = np.array([
    [0.0,  1.0,  0.0],
    [1.0,  0.0,  0.0],
    [0.0,  0.0, -1.0]
], dtype=np.float64)

# R_FLU_FRD: Transform vector/matrix in FRD body frame to FLU body frame
# x_flu = x_frd, y_flu = -y_frd, z_flu = -z_frd
# R_FLU_FRD = R_FRD_FLU (self-inverse)
R_FLU_FRD = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
], dtype=np.float64)


class VOBridgeNode(Node):
    """
    Bridge node converting RTAB-Map nav_msgs/Odometry to px4_msgs/VehicleOdometry.
    """

    def __init__(self):
        super().__init__('vo_bridge')

        # Load parameters
        self._declare_params()
        self._load_params()

        # State tracking
        self._odom_count = 0
        self._pub_count = 0
        self._drop_count = 0
        self._reset_counter = 0
        self._last_pos_ned = None

        # Diagnostic timing
        self._last_diag_time = self.get_clock().now()

        # ── QoS Profiles ──
        # RTAB-Map subscription (best-effort / sensor data)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # PX4 uXRCE-DDS publisher (must be BEST_EFFORT + VOLATILE)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers & Publishers ──
        self._odom_sub = self.create_subscription(
            Odometry,
            self._rtabmap_odom_topic,
            self._odom_callback,
            sub_qos
        )

        self._px4_odom_pub = self.create_publisher(
            VehicleOdometry,
            self._px4_visual_odom_topic,
            px4_qos
        )

        self._diag_pub = self.create_publisher(
            DiagnosticArray,
            '/vo_nav/diagnostics',
            10
        )

        self._diag_timer = self.create_timer(
            self._diag_interval_s,
            self._publish_diagnostics
        )

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  vo_bridge node started')
        self.get_logger().info(f'  Input:  {self._rtabmap_odom_topic} (nav_msgs/Odometry)')
        self.get_logger().info(f'  Output: {self._px4_visual_odom_topic} (px4_msgs/VehicleOdometry)')
        self.get_logger().info('═══════════════════════════════════════════')

    def _declare_params(self):
        """Declare ROS 2 parameters."""
        self.declare_parameter('rtabmap_odom_topic', '/rtabmap/odom')
        self.declare_parameter('px4_visual_odom_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('diag_publish_interval_s', 5.0)
        self.declare_parameter('max_position_jump_m', 2.0)
        self.declare_parameter('fallback_position_std', 0.05)
        self.declare_parameter('fallback_orientation_std', 0.01)
        self.declare_parameter('fallback_velocity_std', 0.05)

    def _load_params(self):
        """Load parameters into node instance."""
        self._rtabmap_odom_topic = self.get_parameter('rtabmap_odom_topic').value
        self._px4_visual_odom_topic = self.get_parameter('px4_visual_odom_topic').value
        self._diag_interval_s = float(self.get_parameter('diag_publish_interval_s').value)
        self._max_pos_jump = float(self.get_parameter('max_position_jump_m').value)
        self._fallback_pos_std = float(self.get_parameter('fallback_position_std').value)
        self._fallback_ori_std = float(self.get_parameter('fallback_orientation_std').value)
        self._fallback_vel_std = float(self.get_parameter('fallback_velocity_std').value)

    def _odom_callback(self, msg: Odometry):
        """
        Process RTAB-Map odometry and convert to PX4 VehicleOdometry.
        """
        self._odom_count += 1

        # Extract timestamp in microseconds for PX4
        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec

        if stamp_sec == 0 and stamp_nanosec == 0:
            # Fallback to current ROS node time if header timestamp is empty
            now = self.get_clock().now()
            ts_us = int(now.nanoseconds // 1000)
        else:
            ts_us = int(stamp_sec * 1_000_000 + stamp_nanosec // 1000)

        # ── 1. Position Conversion (ENU -> NED) ──
        p_enu = msg.pose.pose.position
        if math.isnan(p_enu.x) or math.isnan(p_enu.y) or math.isnan(p_enu.z):
            self._drop_count += 1
            self.get_logger().warn('Received NaN position from RTAB-Map odometry — dropping', throttle_duration_sec=2.0)
            return

        pos_ned = [
            float(p_enu.y),   # North = East_ENU (Y_enu)
            float(p_enu.x),   # East  = North_ENU (X_enu)
            float(-p_enu.z)   # Down  = -Up_ENU (-Z_enu)
        ]

        # Check for unexpected position jump (indicates tracking reset or loop closure)
        if self._last_pos_ned is not None:
            dist_jump = math.sqrt(
                (pos_ned[0] - self._last_pos_ned[0]) ** 2 +
                (pos_ned[1] - self._last_pos_ned[1]) ** 2 +
                (pos_ned[2] - self._last_pos_ned[2]) ** 2
            )
            if dist_jump > self._max_pos_jump:
                self._reset_counter = (self._reset_counter + 1) % 256
                self.get_logger().warn(
                    f'Position jump of {dist_jump:.2f}m detected! Incremented reset_counter to {self._reset_counter}'
                )

        self._last_pos_ned = pos_ned

        # ── 2. Orientation Conversion (ENU/FLU -> NED/FRD) ──
        q_enu = msg.pose.pose.orientation
        if math.isnan(q_enu.x) or math.isnan(q_enu.y) or math.isnan(q_enu.z) or math.isnan(q_enu.w):
            self._drop_count += 1
            return

        # Scipy uses [x, y, z, w] ordering
        r_enu_flu = Rotation.from_quat([q_enu.x, q_enu.y, q_enu.z, q_enu.w])
        mat_enu_flu = r_enu_flu.as_matrix()

        # Compute full rotation matrix: R_NED_FRD = R_NED_ENU * R_ENU_FLU * R_FLU_FRD
        mat_ned_frd = R_NED_ENU @ mat_enu_flu @ R_FLU_FRD
        r_ned_frd = Rotation.from_matrix(mat_ned_frd)
        q_scipy = r_ned_frd.as_quat()  # [x, y, z, w]

        # PX4 expects Hamiltonian quaternion: [w, x, y, z]
        q_px4 = [
            float(q_scipy[3]),  # w
            float(q_scipy[0]),  # x
            float(q_scipy[1]),  # y
            float(q_scipy[2])   # z
        ]

        # ── 3. Velocity Conversion (Body FLU -> Body FRD) ──
        v_flu = msg.twist.twist.linear
        w_flu = msg.twist.twist.angular

        # Body velocity: FLU -> FRD
        vel_frd = [
            float(v_flu.x),    # Front unchanged
            float(-v_flu.y),   # Right = -Left
            float(-v_flu.z)    # Down  = -Up
        ]

        # Body angular velocity: FLU -> FRD
        ang_vel_frd = [
            float(w_flu.x),    # Roll rate unchanged
            float(-w_flu.y),   # Pitch rate = -Pitch
            float(-w_flu.z)    # Yaw rate   = -Yaw
        ]

        # ── 4. Covariances ──
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        # Position variance (X_ned=Y_enu, Y_ned=X_enu, Z_ned=Z_enu)
        if pose_cov[0] > 0 and pose_cov[7] > 0 and pose_cov[14] > 0:
            pos_var = [
                float(pose_cov[7]),   # Var(Y_enu) -> Var(X_ned)
                float(pose_cov[0]),   # Var(X_enu) -> Var(Y_ned)
                float(pose_cov[14])   # Var(Z_enu) -> Var(Z_ned)
            ]
        else:
            pos_var = [self._fallback_pos_std ** 2] * 3

        # Orientation variance
        if pose_cov[21] > 0 and pose_cov[28] > 0 and pose_cov[35] > 0:
            ori_var = [
                float(pose_cov[21]),  # Roll var
                float(pose_cov[28]),  # Pitch var
                float(pose_cov[35])   # Yaw var
            ]
        else:
            ori_var = [self._fallback_ori_std ** 2] * 3

        # Velocity variance
        if twist_cov[0] > 0 and twist_cov[7] > 0 and twist_cov[14] > 0:
            vel_var = [
                float(twist_cov[0]),
                float(twist_cov[7]),
                float(twist_cov[14])
            ]
        else:
            vel_var = [self._fallback_vel_std ** 2] * 3

        # ── 5. Build PX4 VehicleOdometry Message ──
        out_msg = VehicleOdometry()
        out_msg.timestamp = ts_us
        out_msg.timestamp_sample = ts_us

        # Frames
        out_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out_msg.position = pos_ned
        out_msg.q = q_px4

        out_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out_msg.velocity = vel_frd
        out_msg.angular_velocity = ang_vel_frd

        out_msg.position_variance = pos_var
        out_msg.orientation_variance = ori_var
        out_msg.velocity_variance = vel_var

        out_msg.reset_counter = self._reset_counter
        out_msg.quality = 100  # Standard non-zero quality score

        # Publish
        self._px4_odom_pub.publish(out_msg)
        self._pub_count += 1

    def _publish_diagnostics(self):
        """Publish diagnostic status."""
        now = self.get_clock().now()
        dt = (now - self._last_diag_time).nanoseconds / 1e9
        self._last_diag_time = now

        in_rate = self._odom_count / dt if dt > 0 else 0.0
        pub_rate = self._pub_count / dt if dt > 0 else 0.0

        if pub_rate > 0:
            level = DiagnosticStatus.OK
            message = f'VO Bridge Active | In: {in_rate:.1f}Hz | Out: {pub_rate:.1f}Hz'
        else:
            level = DiagnosticStatus.WARN
            message = f'No odometry published! Waiting for {self._rtabmap_odom_topic}...'

        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()

        status = DiagnosticStatus()
        status.name = 'vo_bridge'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='rtabmap_odom_rate_hz', value=f'{in_rate:.1f}'),
            KeyValue(key='px4_odom_pub_rate_hz', value=f'{pub_rate:.1f}'),
            KeyValue(key='dropped_frames', value=str(self._drop_count)),
            KeyValue(key='reset_counter', value=str(self._reset_counter)),
        ]

        diag.status.append(status)
        self._diag_pub.publish(diag)

        self.get_logger().info(f'[VO_BRIDGE] {message}')

        # Reset period counters
        self._odom_count = 0
        self._pub_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = VOBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vo_bridge...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
