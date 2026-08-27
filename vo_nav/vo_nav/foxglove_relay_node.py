"""
foxglove_relay_node — Lightweight image compressor for WiFi visualization.

Problem:
    Raw 640x480 RGB at 30fps ~22 MB/s through DDS.
    Foxglove Bridge forwards ALL of this over WiFi -> frame drops, latency.

Solution:
    Subscribe to raw camera images, compress aggressively, publish as
    CompressedImage on dedicated topics. Foxglove subscribes to these
    instead of the raw streams.

    Color: JPEG at quality ~40  ->  ~15-30 KB/frame  (30-60x compression)
    Depth: PNG 16-bit lossless  ->  ~50-100 KB/frame (5-10x compression)

    Total: ~1.5 MB/s at 30fps — easily fits over WiFi.

Publishes:
    /foxglove/color/compressed  — sensor_msgs/CompressedImage (JPEG)
    /foxglove/depth/compressed  — sensor_msgs/CompressedImage (PNG)

Target: Jetson Orin Nano · ROS 2 Jazzy · Python 3.12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class FoxgloveRelayNode(Node):
    """
    Subscribes to raw camera images and publishes JPEG/PNG compressed
    versions for low-bandwidth Foxglove visualization over WiFi.
    """

    def __init__(self):
        super().__init__('foxglove_relay')

        # ── Parameters ──
        self.declare_parameter('color_topic_in', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic_in', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('infra1_topic_in', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('infra2_topic_in', '/camera/camera/infra2/image_rect_raw')
        self.declare_parameter('color_topic_out', '/foxglove/color/compressed')
        self.declare_parameter('depth_topic_out', '/foxglove/depth/compressed')
        self.declare_parameter('infra1_topic_out', '/foxglove/infra1/compressed')
        self.declare_parameter('infra2_topic_out', '/foxglove/infra2/compressed')
        self.declare_parameter('jpeg_quality', 40)        # 1-100, lower = smaller
        self.declare_parameter('downscale_factor', 1.0)   # 0.5 = half resolution
        self.declare_parameter('max_fps', 15.0)           # throttle output rate
        self.declare_parameter('enable_depth', True)       # depth compression is heavier
        self.declare_parameter('enable_infra', True)       # infra1 + infra2 streams

        self._color_in = self.get_parameter('color_topic_in').value
        self._depth_in = self.get_parameter('depth_topic_in').value
        self._infra1_in = self.get_parameter('infra1_topic_in').value
        self._infra2_in = self.get_parameter('infra2_topic_in').value
        self._color_out = self.get_parameter('color_topic_out').value
        self._depth_out = self.get_parameter('depth_topic_out').value
        self._infra1_out = self.get_parameter('infra1_topic_out').value
        self._infra2_out = self.get_parameter('infra2_topic_out').value
        self._jpeg_quality = self.get_parameter('jpeg_quality').value
        self._downscale = self.get_parameter('downscale_factor').value
        self._max_fps = self.get_parameter('max_fps').value
        self._enable_depth = self.get_parameter('enable_depth').value
        self._enable_infra = self.get_parameter('enable_infra').value

        self._min_interval_ns = int(1e9 / self._max_fps) if self._max_fps > 0 else 0
        self._last_color_ns = 0
        self._last_depth_ns = 0
        self._last_infra1_ns = 0
        self._last_infra2_ns = 0

        self._bridge = CvBridge()

        # ── Counters ──
        self._color_count = 0
        self._depth_count = 0
        self._infra1_count = 0
        self._infra2_count = 0
        self._color_skip = 0
        self._depth_skip = 0
        self._infra1_skip = 0
        self._infra2_skip = 0

        # ── QoS: best-effort, volatile, depth=1 — latest frame only ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──
        self._color_sub = self.create_subscription(
            Image, self._color_in, self._color_cb, sensor_qos
        )

        if self._enable_depth:
            self._depth_sub = self.create_subscription(
                Image, self._depth_in, self._depth_cb, sensor_qos
            )

        if self._enable_infra:
            self._infra1_sub = self.create_subscription(
                Image, self._infra1_in, self._infra1_cb, sensor_qos
            )
            self._infra2_sub = self.create_subscription(
                Image, self._infra2_in, self._infra2_cb, sensor_qos
            )

        # ── Publishers ──
        self._color_pub = self.create_publisher(
            CompressedImage, self._color_out, sensor_qos
        )

        if self._enable_depth:
            self._depth_pub = self.create_publisher(
                CompressedImage, self._depth_out, sensor_qos
            )

        if self._enable_infra:
            self._infra1_pub = self.create_publisher(
                CompressedImage, self._infra1_out, sensor_qos
            )
            self._infra2_pub = self.create_publisher(
                CompressedImage, self._infra2_out, sensor_qos
            )

        # ── Stats timer ──
        self._stats_timer = self.create_timer(10.0, self._log_stats)

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  Foxglove Relay Node')
        self.get_logger().info(f'  Color: {self._color_in} -> {self._color_out}')
        self.get_logger().info(f'         JPEG quality={self._jpeg_quality}, '
                               f'downscale={self._downscale}x')
        if self._enable_depth:
            self.get_logger().info(f'  Depth: {self._depth_in} -> {self._depth_out}')
            self.get_logger().info(f'         PNG 16-bit lossless')
        if self._enable_infra:
            self.get_logger().info(f'  IR-L:  {self._infra1_in} -> {self._infra1_out}')
            self.get_logger().info(f'  IR-R:  {self._infra2_in} -> {self._infra2_out}')
            self.get_logger().info(f'         JPEG quality={self._jpeg_quality}')
        self.get_logger().info(f'  Max FPS: {self._max_fps}')
        self.get_logger().info('═══════════════════════════════════════════')

    # ────────────────────────────────────────────────────────────────────
    #  Color callback — JPEG compression
    # ────────────────────────────────────────────────────────────────────

    def _color_cb(self, msg: Image):
        """Compress color frame to JPEG."""
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_color_ns) < self._min_interval_ns:
            self._color_skip += 1
            return
        self._last_color_ns = now_ns

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self._downscale != 1.0:
                w = int(cv_img.shape[1] * self._downscale)
                h = int(cv_img.shape[0] * self._downscale)
                cv_img = cv2.resize(cv_img, (w, h), interpolation=cv2.INTER_AREA)

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            success, buf = cv2.imencode('.jpg', cv_img, encode_params)
            if not success:
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()

            self._color_pub.publish(out)
            self._color_count += 1

        except Exception as e:
            self.get_logger().error(f'Color compress failed: {e}', throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    #  Depth callback — PNG compression (lossless 16-bit)
    # ────────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        """Compress depth frame to 16-bit PNG (lossless)."""
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_depth_ns) < self._min_interval_ns:
            self._depth_skip += 1
            return
        self._last_depth_ns = now_ns

        try:
            if msg.encoding == '32FC1':
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                cv_img = np.nan_to_num(cv_img, nan=0.0)
                cv_img = (cv_img * 1000.0).clip(0, 65535).astype(np.uint16)
            else:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

            if self._downscale != 1.0:
                w = int(cv_img.shape[1] * self._downscale)
                h = int(cv_img.shape[0] * self._downscale)
                cv_img = cv2.resize(cv_img, (w, h), interpolation=cv2.INTER_NEAREST)

            encode_params = [cv2.IMWRITE_PNG_COMPRESSION, 9]
            success, buf = cv2.imencode('.png', cv_img, encode_params)
            if not success:
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'png'
            out.data = buf.tobytes()

            self._depth_pub.publish(out)
            self._depth_count += 1

        except Exception as e:
            self.get_logger().error(f'Depth compress failed: {e}', throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    #  Infra callbacks — JPEG compression on MONO8 grayscale
    # ────────────────────────────────────────────────────────────────────

    def _infra_compress(self, msg: Image, publisher, last_ns_attr: str,
                        count_attr: str, skip_attr: str):
        """Compress a MONO8 infrared frame to JPEG."""
        now_ns = self.get_clock().now().nanoseconds
        last_ns = getattr(self, last_ns_attr)
        if (now_ns - last_ns) < self._min_interval_ns:
            setattr(self, skip_attr, getattr(self, skip_attr) + 1)
            return
        setattr(self, last_ns_attr, now_ns)

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            if self._downscale != 1.0:
                w = int(cv_img.shape[1] * self._downscale)
                h = int(cv_img.shape[0] * self._downscale)
                cv_img = cv2.resize(cv_img, (w, h), interpolation=cv2.INTER_AREA)

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            success, buf = cv2.imencode('.jpg', cv_img, encode_params)
            if not success:
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()

            publisher.publish(out)
            setattr(self, count_attr, getattr(self, count_attr) + 1)

        except Exception as e:
            self.get_logger().error(f'Infra compress failed: {e}', throttle_duration_sec=5.0)

    def _infra1_cb(self, msg: Image):
        """Compress infra1 (left IR) frame."""
        self._infra_compress(msg, self._infra1_pub,
                             '_last_infra1_ns', '_infra1_count', '_infra1_skip')

    def _infra2_cb(self, msg: Image):
        """Compress infra2 (right IR) frame."""
        self._infra_compress(msg, self._infra2_pub,
                             '_last_infra2_ns', '_infra2_count', '_infra2_skip')

    # ────────────────────────────────────────────────────────────────────
    #  Stats
    # ────────────────────────────────────────────────────────────────────

    def _log_stats(self):
        """Log compression stats every 10s."""
        self.get_logger().info(
            f'[RELAY] Color: {self._color_count} sent, {self._color_skip} throttled | '
            f'Depth: {self._depth_count} sent, {self._depth_skip} throttled | '
            f'IR-L: {self._infra1_count} sent, {self._infra1_skip} throttled | '
            f'IR-R: {self._infra2_count} sent, {self._infra2_skip} throttled'
        )
        self._color_count = 0
        self._depth_count = 0
        self._infra1_count = 0
        self._infra2_count = 0
        self._color_skip = 0
        self._depth_skip = 0
        self._infra1_skip = 0
        self._infra2_skip = 0


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down foxglove_relay...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
