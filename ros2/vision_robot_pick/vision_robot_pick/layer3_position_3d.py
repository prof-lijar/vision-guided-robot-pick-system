#!/usr/bin/env python3
"""
Layer 3 - 3D Position from Point Cloud
Tests : combine YOLO 2D detection with point cloud to get 3D position
Topics: subscribes /camera/color/image_raw
        subscribes /camera/depth/points
        publishes  /object/position_3d  (geometry_msgs/PointStamped)
Run   : ros2 run vision_robot_pick position_3d
        ros2 run vision_robot_pick position_3d --ros-args -p target_class:='cell phone'
        ros2 run vision_robot_pick position_3d --ros-args -p target_class:=''
Pass  : terminal prints XYZ in meters, /object/position_3d echoes real values
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import numpy as np
from ultralytics import YOLO


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)

CONFIDENCE_THRESHOLD = 0.3
MODEL_PATH           = 'yolov8n.pt'
YOLO_SKIP            = 3        # run YOLO every N frames to reduce CPU load

# Camera intrinsics from Layer 1
FX     = 357.99;  FY = 357.99
CX     = 318.69;  CY = 177.58
IMG_W  = 640;     IMG_H = 360
MIN_DEPTH = 0.1   # meters
MAX_DEPTH = 5.0   # meters


class Position3DNode(Node):
    def __init__(self):
        super().__init__('position_3d')
        self.bridge = CvBridge()

        # ── ROS2 parameter: filter by class name ──────────────────────────
        # Empty string = detect ALL classes
        # 'cell phone'  = only detect cell phone (calibration mode)
        # 'bottle'      = only detect bottle, etc.
        self.declare_parameter('target_class', 'cell phone')
        self.target_class = self.get_parameter('target_class').value

        # ── Load YOLO ─────────────────────────────────────────────────────
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded OK')

        # ── Point cloud cache ─────────────────────────────────────────────
        self.xyz         = None    # valid 3D points (N, 3) in meters
        self.u_proj      = None    # projected pixel u coords
        self.v_proj      = None    # projected pixel v coords
        self.pc_frame_id = ''

        # ── Frame counter for YOLO throttling ─────────────────────────────
        self.frame_count = 0

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/camera/depth/points',
            self.cloud_cb, SENSOR_QOS)

        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.color_cb, SENSOR_QOS)

        # ── Publisher ─────────────────────────────────────────────────────
        self.pos_pub = self.create_publisher(
            PointStamped, '/object/position_3d', 10)

        # Add image publisher alongside existing pos_pub
        self.img_pub = self.create_publisher(Image, '/yolo/detected_image', 10)

        mode = f'FILTER: {self.target_class}' if self.target_class else 'ALL classes'
        self.get_logger().info(
            f'\n{"="*52}\n'
            f'  Layer 3: Position3D started\n'
            f'  Mode       : {mode}\n'
            f'  Confidence : {CONFIDENCE_THRESHOLD}\n'
            f'  YOLO skip  : every {YOLO_SKIP} frames\n'
            f'  Depth range: {MIN_DEPTH}m - {MAX_DEPTH}m\n'
            f'{"="*52}\n'
            f'  To change target class:\n'
            f'  ros2 run vision_robot_pick position_3d \\\n'
            f'    --ros-args -p target_class:=\'cell phone\'\n'
            f'  ros2 run vision_robot_pick position_3d \\\n'
            f'    --ros-args -p target_class:=\'\'   (all classes)\n'
            f'{"="*52}'
        )

    # ── Point Cloud Callback ───────────────────────────────────────────────

    def cloud_cb(self, msg: PointCloud2):
        """
        Parse point cloud and project all valid 3D points to pixel space.
        Stored as numpy arrays for fast lookup in color_cb.
        """
        try:
            # Parse raw bytes directly — much faster than pc2.read_points loop
            field_map  = {f.name: f.offset for f in msg.fields}
            x_off      = field_map['x']
            y_off      = field_map['y']
            z_off      = field_map['z']
            point_step = msg.point_step
            n_pts      = msg.width * msg.height

            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                n_pts, point_step)

            xs = np.frombuffer(
                raw[:, x_off:x_off+4].tobytes(), dtype=np.float32)
            ys = np.frombuffer(
                raw[:, y_off:y_off+4].tobytes(), dtype=np.float32)
            zs = np.frombuffer(
                raw[:, z_off:z_off+4].tobytes(), dtype=np.float32)

            # Filter valid depth range
            valid = (
                np.isfinite(zs) &
                np.isfinite(xs) &
                np.isfinite(ys) &
                (zs > MIN_DEPTH) &
                (zs < MAX_DEPTH)
            )

            if not np.any(valid):
                return

            xv = xs[valid]
            yv = ys[valid]
            zv = zs[valid]

            # Project to pixel space using camera intrinsics
            self.u_proj      = (xv * FX / zv) + CX
            self.v_proj      = (yv * FY / zv) + CY
            self.xyz         = np.stack([xv, yv, zv], axis=1)
            self.pc_frame_id = msg.header.frame_id

        except Exception as e:
            self.get_logger().warn(f'cloud_cb error: {e}')

    # ── 3D Lookup ──────────────────────────────────────────────────────────

    def get_xyz_at_pixel(self, u: int, v: int, radius: int = 8):
        """
        Find 3D position at image pixel (u, v).
        Projects all point cloud points to pixel space, finds nearby points,
        returns median XYZ for robustness against noise.
        Returns (x, y, z) in meters or None if no valid points found.
        """
        if self.xyz is None:
            return None

        # Find points within radius pixels of target (u, v)
        dist_sq = (self.u_proj - u)**2 + (self.v_proj - v)**2
        nearby  = dist_sq < (radius ** 2)

        if not np.any(nearby):
            # Fallback: use single closest point
            idx = np.argmin(dist_sq)
            pts = self.xyz[idx]
            return (float(pts[0]), float(pts[1]), float(pts[2]))

        # Median of nearby points — robust against depth noise
        pts = self.xyz[nearby]
        return (
            float(np.median(pts[:, 0])),
            float(np.median(pts[:, 1])),
            float(np.median(pts[:, 2]))
        )

    # ── Color Image Callback ───────────────────────────────────────────────

    def color_cb(self, msg: Image):
        if self.xyz is None:
            return

        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Throttle YOLO — skip frames to reduce CPU load
        self.frame_count += 1
        if self.frame_count % YOLO_SKIP != 0:
            cv2.imshow('Layer3: 3D Position', color)
            cv2.waitKey(1)
            return

        results = self.model(color, verbose=False)

        detected_any = False

        for box in results[0].boxes:
            conf = float(box.conf)
            if conf < CONFIDENCE_THRESHOLD:
                continue

            cls_name = self.model.names[int(box.cls)]

            # ── Class filter ──────────────────────────────────────────────
            if self.target_class and cls_name != self.target_class:
                # Draw ignored detections in grey
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(color, (x1, y1), (x2, y2), (128, 128, 128), 1)
                cv2.putText(color, f'{cls_name} (ignored)',
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (128, 128, 128), 1)
                continue

            detected_any = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # ── Get 3D position ───────────────────────────────────────────
            xyz = self.get_xyz_at_pixel(cx, cy)

            if xyz is None:
                self.get_logger().warn(
                    f'  [{cls_name}] conf={conf:.2f} | '
                    f'pixel=({cx},{cy}) | NO DEPTH')

                # Red box = no depth
                cv2.rectangle(color, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(color, f'{cls_name} NO DEPTH',
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 0, 255), 2)
                continue

            x, y, z = xyz

            self.get_logger().info(
                f'  [{cls_name}] conf={conf:.2f} | '
                f'pixel=({cx},{cy}) | '
                f'X={x:.3f}m  Y={y:.3f}m  Z={z:.3f}m'
            )

            # ── Publish ───────────────────────────────────────────────────
            pt_msg                 = PointStamped()
            pt_msg.header          = msg.header
            pt_msg.header.frame_id = self.pc_frame_id
            pt_msg.point.x         = x
            pt_msg.point.y         = y
            pt_msg.point.z         = z
            self.pos_pub.publish(pt_msg)

            # ── Draw green box + depth label ──────────────────────────────
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

            label = f'{cls_name} {conf:.2f} | Z={z:.2f}m'
            lsz, _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(color,
                          (x1, y1 - lsz[1] - 10),
                          (x1 + lsz[0], y1),
                          (0, 255, 0), -1)
            cv2.putText(color, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

        # ── HUD: mode indicator ───────────────────────────────────────────
        mode_text = (f'Target: {self.target_class}'
                     if self.target_class else 'Target: ALL')
        cv2.putText(color, mode_text,
                    (10, IMG_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 2)

        if not detected_any and self.target_class:
            cv2.putText(color,
                        f'Searching for: {self.target_class}...',
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 165, 255), 2)

        # Publish annotated frame to Unity
        annotated_msg = self.bridge.cv2_to_imgmsg(color, encoding = 'bgr8')
        annotated_msg.header = msg.header
        self.img_pub.publish(annotated_msg)

        cv2.imshow('Layer3: 3D Position', color)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = Position3DNode()
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
