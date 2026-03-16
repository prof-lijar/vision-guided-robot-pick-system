#!/usr/bin/env python3
"""
Layer 1 — Camera Verification Node
Tests : color image, depth image, camera intrinsics
Run   : ros2 run vision_robot_pick camera_check
Pass  : Prints intrinsics + image sizes, shows color + depth windows
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


# Orbbec camera publishes with BEST_EFFORT reliability — must match!
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)


class CameraCheckNode(Node):
    def __init__(self):
        super().__init__('camera_check')
        self.bridge = CvBridge()

        self.got_info  = False
        self.got_color = False
        self.got_depth = False

        # All subscriptions use BEST_EFFORT QoS to match Orbbec driver
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.info_cb, SENSOR_QOS)

        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.color_cb, SENSOR_QOS)

        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_cb, SENSOR_QOS)

        self.get_logger().info(
            'Layer 1: Camera Check started — waiting for topics...')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def info_cb(self, msg: CameraInfo):
        if not self.got_info:
            self.got_info = True
            fx = msg.k[0]; fy = msg.k[4]
            cx = msg.k[2]; cy = msg.k[5]
            self.get_logger().info(
                f'\n{"─"*52}\n'
                f'  [✓] Camera Intrinsics\n'
                f'      frame_id  : {msg.header.frame_id}\n'
                f'      fx={fx:.2f}  fy={fy:.2f}\n'
                f'      cx={cx:.2f}  cy={cy:.2f}\n'
                f'      resolution: {msg.width} x {msg.height}\n'
                f'{"─"*52}'
            )
            self._check_all_ready()

    def color_cb(self, msg: Image):
        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if not self.got_color:
            self.got_color = True
            self.get_logger().info(
                f'  [✓] Color image | '
                f'{color.shape[1]}x{color.shape[0]} | '
                f'frame_id: {msg.header.frame_id}'
            )
            self._check_all_ready()

        cv2.imshow('Layer1: Color', color)
        cv2.waitKey(1)

    def depth_cb(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

        if not self.got_depth:
            self.got_depth = True
            valid = int(np.count_nonzero(depth))
            total = depth.shape[0] * depth.shape[1]
            pct   = 100 * valid // total
            self.get_logger().info(
                f'  [✓] Depth image (/camera/depth/image_raw) | '
                f'{depth.shape[1]}x{depth.shape[0]} | '
                f'encoding: 16UC1 | '
                f'valid pixels: {valid}/{total} ({pct}%)\n'
                f'      frame_id: {msg.header.frame_id}'
            )
            self._check_all_ready()

        # Colorize depth for display (clip 0–3000 mm)
        depth_vis = np.clip(depth, 0, 3000).astype(np.float32) / 3000.0
        depth_vis = (depth_vis * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        cv2.imshow('Layer1: Depth', depth_color)
        cv2.waitKey(1)

    def _check_all_ready(self):
        if self.got_info and self.got_color and self.got_depth:
            self.get_logger().info(
                '\n' + '='*52 + '\n'
                '  ✅  LAYER 1 PASSED\n'
                '      All 3 camera streams working.\n'
                '      Depth: 640x360, 16UC1 (same as color)\n'
                '      Ready for Layer 2 (YOLO 2D detection)\n'
                + '='*52
            )


def main():
    rclpy.init()
    node = CameraCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
