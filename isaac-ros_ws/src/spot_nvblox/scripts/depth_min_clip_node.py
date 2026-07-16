#!/usr/bin/env python3
"""Clip near-range depth pixels so they aren't fused into nvblox.

Why this exists
---------------
Real depth cameras (RealSense, ZED) have a minimum range (~0.3–0.5 m) and
return *invalid* (zero) below it. Isaac Sim's rendered depth is valid all the
way to the lens, so when the gripper closes and the jaw/grasped object fills the
hand camera frame, that few-centimeter geometry is real depth and nvblox fuses
it — polluting the static map with a phantom blob right where the hand is. This
never happens on the real robot.

nvblox treats any depth <= 0 as invalid and skips it
(projective_tsdf_integrator.cu), and its min projection depth is effectively
zero (1e-6), so there is no launch param to gate near depth. This node mimics
the real sensor: it subscribes to a depth image, zeros every pixel closer than
`min_range_m` (and, optionally, farther than `max_range_m`), and republishes.
Point nvblox's depth input at the clipped topic.

Supports 32FC1 (meters) and 16UC1 (millimeters) depth encodings. The header,
encoding, and dimensions are preserved so camera_info / TF stay valid.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class DepthMinClipNode(Node):
    def __init__(self):
        super().__init__('depth_min_clip_node')

        self.declare_parameter('input_topic', '/depth_registered/hand/image')
        self.declare_parameter('output_topic', '/depth_registered/hand/image_clipped')
        self.declare_parameter('min_range_m', 0.3)   # below this -> invalid (0), like a real sensor
        self.declare_parameter('max_range_m', 0.0)   # 0 disables the far clip

        self._in = self.get_parameter('input_topic').get_parameter_value().string_value
        self._out = self.get_parameter('output_topic').get_parameter_value().string_value
        self._min_m = self.get_parameter('min_range_m').get_parameter_value().double_value
        self._max_m = self.get_parameter('max_range_m').get_parameter_value().double_value

        # SENSOR_DATA-style QoS to match camera publishers / nvblox input.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._pub = self.create_publisher(Image, self._out, qos)
        self._sub = self.create_subscription(Image, self._in, self._on_depth, qos)

        self._warned_encoding = False
        self.get_logger().info(
            f'Depth min-clip: {self._in} -> {self._out} | '
            f'min_range={self._min_m:.3f} m'
            + (f', max_range={self._max_m:.3f} m' if self._max_m > 0.0 else '')
        )

    def _on_depth(self, msg: Image):
        enc = msg.encoding.lower()
        if enc in ('32fc1', '32fc'):
            depth = np.frombuffer(msg.data, dtype=np.float32).copy()
            min_v = self._min_m
            max_v = self._max_m
        elif enc in ('16uc1', 'mono16'):
            depth = np.frombuffer(msg.data, dtype=np.uint16).astype(np.uint16).copy()
            min_v = self._min_m * 1000.0   # mm
            max_v = self._max_m * 1000.0
        else:
            if not self._warned_encoding:
                self.get_logger().warn(
                    f'Unsupported depth encoding "{msg.encoding}"; passing through unchanged.'
                )
                self._warned_encoding = True
            self._pub.publish(msg)
            return

        # Zero out near pixels (and far, if enabled). NaN stays NaN (already invalid).
        # `< min` is the real-sensor min-range behavior; nvblox skips depth <= 0.
        near = depth < min_v
        depth[near] = 0
        if max_v > 0.0:
            depth[depth > max_v] = 0

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = depth.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMinClipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
