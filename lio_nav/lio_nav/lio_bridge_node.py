"""
lio_bridge_node — Bridge Point-LIO Odometry to PX4 EKF2 with Initial Pose Alignment

Target: Jetson Orin Nano running ROS 2 Jazzy

Features:
  1. Automatic Initial Pose Zeroing (OdomCorrector):
     - Captures the initial random pose (p0, r0) on the first received odometry message.
     - Transforms all subsequent odometry into a zeroed 'odom_fixed' frame where
       'odom_fixed' and 'base_link' completely overlap at launch.
  2. Publishes corrected odometry on /odom_corrected (nav_msgs/Odometry).
  3. Broadcasts TF transform for odom_fixed -> base_link via TransformBroadcaster.
  4. Translates corrected odometry into PX4 VehicleOdometry (/fmu/in/vehicle_visual_odometry).
  5. Diagnostic reporting & health checks.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from px4_msgs.msg import VehicleOdometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Matrix mapping zeroed FLU world frame (X=Forward at launch, Y=Left, Z=Up) to PX4 NED frame
# X_ned = X_flu (North/Forward), Y_ned = -Y_flu (East/Right), Z_ned = -Z_flu (Down)
R_NED_FLU = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
], dtype=np.float64)

# Matrix mapping FRD body frame to FLU body frame
R_FLU_FRD = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
], dtype=np.float64)


class LIOBridgeNode(Node):
    """
    Bridge node converting Point-LIO nav_msgs/Odometry to PX4 VehicleOdometry
    with initial pose correction (odom_fixed).
    """

    def __init__(self):
        super().__init__('lio_bridge')

        # Load parameters
        self._declare_params()
        self._load_params()

        # State tracking
        self._initialized = False
        self._p0 = None
        self._r0 = None
        self._r0_inv = None

        self._odom_count = 0
        self._pub_count = 0
        self._drop_count = 0
        self._reset_counter = 0
        self._last_pos_ned = None

        # Diagnostic timing
        self._last_diag_time = self.get_clock().now()

        # TF Broadcaster
        self._tf_br = TransformBroadcaster(self)

        # ── QoS Profiles ──
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers & Publishers ──
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            sub_qos
        )

        self._odom_corrected_pub = self.create_publisher(
            Odometry,
            self._odom_corrected_topic,
            10
        )

        self._px4_odom_pub = self.create_publisher(
            VehicleOdometry,
            self._px4_visual_odom_topic,
            px4_qos
        )

        self._diag_pub = self.create_publisher(
            DiagnosticArray,
            '/lio_nav/diagnostics',
            10
        )

        self._diag_timer = self.create_timer(
            self._diag_interval_s,
            self._publish_diagnostics
        )

        self.get_logger().info('════════════════════════════════════════════════════════')
        self.get_logger().info('  lio_bridge node with OdomCorrector started')
        self.get_logger().info(f'  Input Topic:      {self._odom_topic} (nav_msgs/Odometry)')
        self.get_logger().info(f'  Corrected Topic:  {self._odom_corrected_topic} (nav_msgs/Odometry)')
        self.get_logger().info(f'  PX4 Output Topic: {self._px4_visual_odom_topic} (px4_msgs/VehicleOdometry)')
        self.get_logger().info(f'  Fixed Frame ID:   {self._corrected_frame_id}')
        self.get_logger().info('════════════════════════════════════════════════════════')

    def _declare_params(self):
        """Declare ROS 2 parameters."""
        self.declare_parameter('odom_topic', '/odom_corrected')
        self.declare_parameter('odom_corrected_topic', '/odom_corrected_transformed')
        self.declare_parameter('corrected_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link_transformed')
        self.declare_parameter('px4_visual_odom_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('input_is_ned', True)
        self.declare_parameter('diag_publish_interval_s', 5.0)
        self.declare_parameter('max_position_jump_m', 2.0)
        self.declare_parameter('fallback_position_std', 0.05)
        self.declare_parameter('fallback_orientation_std', 0.01)
        self.declare_parameter('fallback_velocity_std', 0.05)

    def _load_params(self):
        """Load parameters into node instance."""
        self._odom_topic = self.get_parameter('odom_topic').value
        self._odom_corrected_topic = self.get_parameter('odom_corrected_topic').value
        self._corrected_frame_id = self.get_parameter('corrected_frame_id').value
        self._child_frame_id = self.get_parameter('child_frame_id').value
        self._px4_visual_odom_topic = self.get_parameter('px4_visual_odom_topic').value
        self._input_is_ned = bool(self.get_parameter('input_is_ned').value)
        self._diag_interval_s = float(self.get_parameter('diag_publish_interval_s').value)
        self._max_pos_jump = float(self.get_parameter('max_position_jump_m').value)
        self._fallback_pos_std = float(self.get_parameter('fallback_position_std').value)
        self._fallback_ori_std = float(self.get_parameter('fallback_orientation_std').value)
        self._fallback_vel_std = float(self.get_parameter('fallback_velocity_std').value)

    def _odom_callback(self, msg: Odometry):
        """
        Process incoming Odometry, align initial transform, publish /odom_corrected, TF, and PX4 VehicleOdometry.
        """
        self._odom_count += 1

        # Extract timestamps
        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec

        if stamp_sec == 0 and stamp_nanosec == 0:
            now = self.get_clock().now()
            ts_us = int(now.nanoseconds // 1000)
        else:
            ts_us = int(stamp_sec * 1_000_000 + stamp_nanosec // 1000)

        # Extract raw position & orientation
        p_raw = msg.pose.pose.position
        q_raw = msg.pose.pose.orientation

        if math.isnan(p_raw.x) or math.isnan(p_raw.y) or math.isnan(p_raw.z):
            self._drop_count += 1
            self.get_logger().warn('Received NaN position from odometry — dropping', throttle_duration_sec=2.0)
            return

        if math.isnan(q_raw.x) or math.isnan(q_raw.y) or math.isnan(q_raw.z) or math.isnan(q_raw.w):
            self._drop_count += 1
            return

        p = np.array([p_raw.x, p_raw.y, p_raw.z], dtype=np.float64)
        q = np.array([q_raw.x, q_raw.y, q_raw.z, q_raw.w], dtype=np.float64)
        rot = Rotation.from_quat(q)

        # ── 1. Initial Pose Capture (OdomCorrector) ──
        if not self._initialized:
            self._p0 = p.copy()
            self._r0 = rot
            self._r0_inv = self._r0.inv()
            self._initialized = True

            euler_deg = self._r0.as_euler('xyz', degrees=True)
            self.get_logger().info('════════════════════════════════════════════════════════')
            self.get_logger().info('  [OdomCorrector] Initial Odometry Pose Captured!')
            self.get_logger().info(f'  p0: [{self._p0[0]:.3f}, {self._p0[1]:.3f}, {self._p0[2]:.3f}]')
            self.get_logger().info(f'  r0 (RPY deg): [{euler_deg[0]:.2f}°, {euler_deg[1]:.2f}°, {euler_deg[2]:.2f}°]')
            self.get_logger().info(f'  Base link and {self._corrected_frame_id} now 100% aligned.')
            self.get_logger().info('════════════════════════════════════════════════════════')

        # ── 2. Pose Correction (relative to odom_fixed) ──
        p_corr = self._r0_inv.apply(p - self._p0)
        r_corr = self._r0_inv * rot
        q_corr = r_corr.as_quat()  # [x, y, z, w]

        v_raw = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ], dtype=np.float64)

        w_raw = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ], dtype=np.float64)

        v_corr = self._r0_inv.apply(v_raw)
        w_corr = self._r0_inv.apply(w_raw)

        # ── 3. Publish Corrected ROS 2 Odometry & TF ──
        out_odom = Odometry()
        out_odom.header = msg.header
        out_odom.header.frame_id = self._corrected_frame_id
        out_odom.child_frame_id = self._child_frame_id

        out_odom.pose.pose.position.x = float(p_corr[0])
        out_odom.pose.pose.position.y = float(p_corr[1])
        out_odom.pose.pose.position.z = float(p_corr[2])

        out_odom.pose.pose.orientation.x = float(q_corr[0])
        out_odom.pose.pose.orientation.y = float(q_corr[1])
        out_odom.pose.pose.orientation.z = float(q_corr[2])
        out_odom.pose.pose.orientation.w = float(q_corr[3])

        out_odom.pose.covariance = msg.pose.covariance

        out_odom.twist.twist.linear.x = float(v_corr[0])
        out_odom.twist.twist.linear.y = float(v_corr[1])
        out_odom.twist.twist.linear.z = float(v_corr[2])

        out_odom.twist.twist.angular.x = float(w_corr[0])
        out_odom.twist.twist.angular.y = float(w_corr[1])
        out_odom.twist.twist.angular.z = float(w_corr[2])

        out_odom.twist.covariance = msg.twist.covariance

        self._odom_corrected_pub.publish(out_odom)

        # Broadcast TF (odom_fixed -> base_link)
        tf_msg = TransformStamped()
        tf_msg.header = out_odom.header
        tf_msg.child_frame_id = out_odom.child_frame_id
        tf_msg.transform.translation.x = out_odom.pose.pose.position.x
        tf_msg.transform.translation.y = out_odom.pose.pose.position.y
        tf_msg.transform.translation.z = out_odom.pose.pose.position.z
        tf_msg.transform.rotation = out_odom.pose.pose.orientation

        self._tf_br.sendTransform(tf_msg)

        # ── 4. Convert to PX4 VehicleOdometry (NED/FRD) ──
        if self._input_is_ned:
            # Input is ALREADY in NED frame (X=North, Y=East, Z=Down)
            pos_ned = [
                float(p_corr[0]),    # North
                float(p_corr[1]),    # East
                float(p_corr[2])     # Down
            ]

            q_px4 = [
                float(q_corr[3]),    # w
                float(q_corr[0]),    # x
                float(q_corr[1]),    # y
                float(q_corr[2])     # z
            ]

            vel_frd = [
                float(v_corr[0]),    # Front/X
                float(v_corr[1]),    # Right/Y
                float(v_corr[2])     # Down/Z
            ]

            ang_vel_frd = [
                float(w_corr[0]),
                float(w_corr[1]),
                float(w_corr[2])
            ]
        else:
            # Input is in standard FLU body / ENU world frame -> transform to NED/FRD
            pos_ned = [
                float(p_corr[0]),    # North = X_corr (Forward)
                float(-p_corr[1]),   # East  = -Y_corr (Right)
                float(-p_corr[2])    # Down  = -Z_corr (Down)
            ]

            mat_ned_frd = R_NED_FLU @ r_corr.as_matrix() @ R_FLU_FRD
            r_ned_frd = Rotation.from_matrix(mat_ned_frd)
            q_scipy = r_ned_frd.as_quat()  # [x, y, z, w]

            q_px4 = [
                float(q_scipy[3]),  # w
                float(q_scipy[0]),  # x
                float(q_scipy[1]),  # y
                float(q_scipy[2])   # z
            ]

            vel_frd = [
                float(v_raw[0]),    # Front unchanged
                float(-v_raw[1]),   # Right = -Left
                float(-v_raw[2])    # Down  = -Up
            ]

            ang_vel_frd = [
                float(w_raw[0]),    # Roll rate unchanged
                float(-w_raw[1]),   # Pitch rate = -Pitch
                float(-w_raw[2])    # Yaw rate   = -Yaw
            ]

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

        # ── 5. Covariance Mapping ──
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        if pose_cov[0] > 0 and pose_cov[7] > 0 and pose_cov[14] > 0:
            pos_var = [
                float(pose_cov[0]),
                float(pose_cov[7]),
                float(pose_cov[14])
            ]
        else:
            pos_var = [self._fallback_pos_std ** 2] * 3

        if pose_cov[21] > 0 and pose_cov[28] > 0 and pose_cov[35] > 0:
            ori_var = [
                float(pose_cov[21]),
                float(pose_cov[28]),
                float(pose_cov[35])
            ]
        else:
            ori_var = [self._fallback_ori_std ** 2] * 3

        if twist_cov[0] > 0 and twist_cov[7] > 0 and twist_cov[14] > 0:
            vel_var = [
                float(twist_cov[0]),
                float(twist_cov[7]),
                float(twist_cov[14])
            ]
        else:
            vel_var = [self._fallback_vel_std ** 2] * 3

        # ── 6. Build & Publish PX4 VehicleOdometry ──
        out_msg = VehicleOdometry()
        out_msg.timestamp = ts_us
        out_msg.timestamp_sample = ts_us

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
        out_msg.quality = 100

        self._px4_odom_pub.publish(out_msg)
        self._pub_count += 1

    def _publish_diagnostics(self):
        now = self.get_clock().now()
        dt = (now - self._last_diag_time).nanoseconds / 1e9
        self._last_diag_time = now

        in_rate = self._odom_count / dt if dt > 0 else 0.0
        pub_rate = self._pub_count / dt if dt > 0 else 0.0

        if pub_rate > 0:
            level = DiagnosticStatus.OK
            message = f'LIO Bridge Active | Initialized: {self._initialized} | In: {in_rate:.1f}Hz | Out: {pub_rate:.1f}Hz'
        else:
            level = DiagnosticStatus.WARN
            message = f'No odometry published! Waiting for {self._odom_topic}...'

        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()

        status = DiagnosticStatus()
        status.name = 'lio_bridge'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='odom_rate_hz', value=f'{in_rate:.1f}'),
            KeyValue(key='px4_odom_pub_rate_hz', value=f'{pub_rate:.1f}'),
            KeyValue(key='initialized', value=str(self._initialized)),
            KeyValue(key='dropped_frames', value=str(self._drop_count)),
            KeyValue(key='reset_counter', value=str(self._reset_counter)),
        ]

        diag.status.append(status)
        self._diag_pub.publish(diag)

        self.get_logger().info(f'[LIO_BRIDGE] {message}')

        self._odom_count = 0
        self._pub_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = LIOBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down lio_bridge...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
