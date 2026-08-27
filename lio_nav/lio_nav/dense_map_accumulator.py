#!/usr/bin/env python3
"""
Standalone Dense Map Accumulator Node for Jetson / ROS 2
=========================================================
1. Subscribes to Point-LIO's registered scan topic (/cloud_registered).
2. Accumulates points into a 5cm voxel grid and publishes dense map on (/dense_map) every 2.0s.
3. Transforms/publishes unilidar scan in base_link frame at 4 Hz on (/unilidar_scan_base_link).
4. Tracks base_link movement via TF and publishes trajectory path on (/path).

"""

import os
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros


def read_points_numpy(msg: PointCloud2) -> np.ndarray:
    """Fast numpy reader for PointCloud2 message -> returns Nx4 (x,y,z,intensity) float32."""
    field_map = {f.name: f.offset for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return np.empty((0, 4), dtype=np.float32)

    point_step = msg.point_step
    n_points = msg.width * msg.height
    if n_points == 0:
        return np.empty((0, 4), dtype=np.float32)

    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)

    x_off = field_map['x']
    y_off = field_map['y']
    z_off = field_map['z']

    x = raw[:, x_off:x_off+4].copy().view(np.float32).ravel()
    y = raw[:, y_off:y_off+4].copy().view(np.float32).ravel()
    z = raw[:, z_off:z_off+4].copy().view(np.float32).ravel()

    if 'intensity' in field_map:
        i_off = field_map['intensity']
        intensity = raw[:, i_off:i_off+4].copy().view(np.float32).ravel()
    else:
        intensity = np.zeros(n_points, dtype=np.float32)

    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return np.column_stack((x[valid], y[valid], z[valid], intensity[valid])).astype(np.float32)


def create_cloud_msg(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Pack Nx4 float32 numpy array into sensor_msgs/PointCloud2."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx*qy - qz*qw)
    r02 = 2 * (qx*qz + qy*qw)

    r10 = 2 * (qx*qy + qz*qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy*qz - qx*qw)

    r20 = 2 * (qx*qz - qy*qw)
    r21 = 2 * (qy*qz + qx*qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]], dtype=np.float32)


class DenseMapAccumulator(Node):
    def __init__(self):
        super().__init__('dense_map_accumulator')

        # Parameters
        self.declare_parameter('voxel_size', 0.05)             # 5cm voxel resolution
        self.declare_parameter('pub_interval', 2.0)             # Publish dense map every 2s
        self.declare_parameter('body_pub_rate', 4.0)            # 4 Hz rate for unilidar/body scan
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/dense_map')
        self.declare_parameter('body_scan_topic', '/unilidar_scan_base_link')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('min_path_dist', 0.05)           # Minimum movement (5cm) to record new path pose
        self.declare_parameter('target_body_frame', 'base_link')
        self.declare_parameter('invert_map', False)            # False = normal orientation
        self.declare_parameter('save_dir', '/home/robot/robocon_ws/saved_maps')

        self.voxel_size = self.get_parameter('voxel_size').value
        self.pub_interval = self.get_parameter('pub_interval').value
        self.body_pub_rate = self.get_parameter('body_pub_rate').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.body_scan_topic = self.get_parameter('body_scan_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.min_path_dist = self.get_parameter('min_path_dist').value
        self.target_body_frame = self.get_parameter('target_body_frame').value
        self.invert_map = self.get_parameter('invert_map').value
        self.save_dir = self.get_parameter('save_dir').value

        self.inv_voxel = 1.0 / self.voxel_size
        self.frame_id = None

        # Voxel grid hash table: voxel_key -> point [x, y, z, intensity]
        self.voxel_map = {}
        self.scan_count = 0
        self.has_new_points = False

        # Path tracking
        self.path_msg = Path()
        self.last_path_pos = None

        # Throttling for body scan output (4 Hz -> 0.25s)
        self.body_pub_interval = 1.0 / self.body_pub_rate if self.body_pub_rate > 0 else 0.25
        self.last_body_pub_time = 0.0

        # TF Listener for transforming world scan to base_link and tracking path
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)

        self.map_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.body_pub = self.create_publisher(PointCloud2, self.body_scan_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.timer = self.create_timer(self.pub_interval, self.publish_map_callback)

        self.get_logger().info(
            f'Dense Map Accumulator Started:\n'
            f'  Input Topic:        {self.input_topic}\n'
            f'  Map Output Topic:   {self.output_topic} (Inverted: {self.invert_map})\n'
            f'  Unilidar Scan Topic:{self.body_scan_topic} ({self.target_body_frame} @ {self.body_pub_rate} Hz)\n'
            f'  Path Output Topic:  {self.path_topic}\n'
            f'  Voxel Size:         {self.voxel_size} m\n'
            f'  Map Pub Interval:   {self.pub_interval} s'
        )

    def cloud_callback(self, msg: PointCloud2):
        if self.frame_id is None:
            self.frame_id = msg.header.frame_id
            self.path_msg.header.frame_id = self.frame_id
            self.get_logger().info(f'Detected world frame_id: {self.frame_id}')

        pts = read_points_numpy(msg)
        if len(pts) == 0:
            return

        # 1. Update and publish path
        self.update_and_publish_path(msg.header)

        # 2. Publish unilidar scan in target_body_frame (base_link) at 4 Hz
        now_sec = time.time()
        if (now_sec - self.last_body_pub_time) >= self.body_pub_interval:
            self.publish_current_body_scan(pts, msg.header)
            self.last_body_pub_time = now_sec

        # 3. Accumulate points into voxel map
        v_keys = np.floor(pts[:, :3] * self.inv_voxel).astype(np.int32)
        v_keys, unique_idx = np.unique(v_keys, axis=0, return_index=True)
        pts = pts[unique_idx]

        for i in range(len(pts)):
            key = (int(v_keys[i, 0]), int(v_keys[i, 1]), int(v_keys[i, 2]))
            if key not in self.voxel_map:
                self.voxel_map[key] = pts[i]
                self.has_new_points = True

        self.scan_count += 1
        if self.scan_count % 50 == 0:
            self.get_logger().info(
                f'Processed {self.scan_count} scans. Total accumulated voxels: {len(self.voxel_map)}')

    def update_and_publish_path(self, header: Header):
        """Lookup robot position via TF in world frame (camera_init) and publish trajectory path."""
        try:
            # Lookup TF: world frame (camera_init) <- target_body_frame (base_link)
            t = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.target_body_frame,
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            # Check distance threshold before appending new pose
            should_add = False
            if self.last_path_pos is None:
                should_add = True
            else:
                dist = math.sqrt((x - self.last_path_pos[0])**2 +
                                 (y - self.last_path_pos[1])**2 +
                                 (z - self.last_path_pos[2])**2)
                if dist >= self.min_path_dist:
                    should_add = True

            if should_add:
                ps = PoseStamped()
                ps.header.stamp = header.stamp
                ps.header.frame_id = self.frame_id
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = z
                ps.pose.orientation = t.transform.rotation

                self.path_msg.header.stamp = header.stamp
                self.path_msg.poses.append(ps)
                self.last_path_pos = (x, y, z)
                self.path_pub.publish(self.path_msg)

        except Exception:
            pass  # TF lookup not ready yet

    def publish_current_body_scan(self, pts: np.ndarray, header: Header):
        """Transform current scan from world frame to base_link frame and publish at 4 Hz as-is."""
        try:
            # Lookup TF: target_body_frame (base_link) <- frame_id (camera_init)
            t = self.tf_buffer.lookup_transform(
                self.target_body_frame,
                header.frame_id,
                rclpy.time.Time()
            )

            tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
            qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w

            R = quat_to_rot_matrix(qx, qy, qz, qw)
            T = np.array([tx, ty, tz], dtype=np.float32)

            body_pts = pts.copy()
            body_pts[:, :3] = body_pts[:, :3] @ R.T + T

            body_msg = create_cloud_msg(body_pts, self.target_body_frame, header.stamp)
            self.body_pub.publish(body_msg)
        except Exception:
            # Fallback if TF lookup is not available yet: publish scan directly in target frame
            body_msg = create_cloud_msg(pts, self.target_body_frame, header.stamp)
            self.body_pub.publish(body_msg)

    def publish_map_callback(self):
        if not self.voxel_map or self.frame_id is None:
            return

        if not self.has_new_points:
            return

        all_points = np.array(list(self.voxel_map.values()), dtype=np.float32)

        # Invert map if enabled
        if self.invert_map:
            all_points[:, :3] *= -1.0

        msg = create_cloud_msg(all_points, self.frame_id, self.get_clock().now().to_msg())
        self.map_pub.publish(msg)
        self.has_new_points = False

        self.get_logger().info(f'Published /dense_map ({len(all_points)} points)')

    def save_map(self):
        if not self.voxel_map:
            return
        os.makedirs(self.save_dir, exist_ok=True)
        pts = np.array(list(self.voxel_map.values()), dtype=np.float32)
        if self.invert_map:
            pts[:, :3] *= -1.0
        n = len(pts)
        out_path = os.path.join(self.save_dir, 'dense_map.pcd')

        with open(out_path, 'wb') as f:
            header = (
                f"# .PCD v0.7 - Point Cloud Data file format\n"
                f"VERSION 0.7\n"
                f"FIELDS x y z intensity\n"
                f"SIZE 4 4 4 4\n"
                f"TYPE F F F F\n"
                f"COUNT 1 1 1 1\n"
                f"WIDTH {n}\n"
                f"HEIGHT 1\n"
                f"VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {n}\n"
                f"DATA binary\n"
            )
            f.write(header.encode('ascii'))
            f.write(pts.tobytes())
        self.get_logger().info(f'Saved dense map ({n} points) to {out_path}')

    def destroy_node(self):
        self.save_map()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DenseMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
