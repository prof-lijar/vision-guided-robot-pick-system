#!/usr/bin/env python3
"""
Layer 3 - 3D Position from Point Cloud (updated)
- Detects ALL YOLO classes
- Publishes /detections as JSON String (label + camera XYZ + robot XY)
- Publishes /yolo/detected_image for Unity
- Publishes /object/position_3d (PointStamped) for backwards compatibility

Topics published:
  /detections            std_msgs/String  — JSON array of all current detections
  /yolo/detected_image   sensor_msgs/Image — annotated camera frame
  /object/position_3d    geometry_msgs/PointStamped — latest detection (compat)

Run:
  ros2 run vision_robot_pick position_3d
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import yaml
import os
from ultralytics import YOLO


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)

CONFIDENCE_THRESHOLD = 0.3
MODEL_PATH           = 'yolov8n.pt'
YOLO_SKIP            = 3

# Camera intrinsics
FX = 357.99;  FY = 357.99
CX = 318.69;  CY = 177.58
IMG_W = 640;  IMG_H = 360
MIN_DEPTH = 0.1
MAX_DEPTH = 5.0

# Calibration file
CALIB_FILE = os.path.expanduser(
    '~/ros2_ws/src/vision_robot_pick/config/calibration.yaml')

# Correction offsets (measured from ground truth)
OFFSET_X_MM = 6.963
OFFSET_Y_MM = 38.277

# Fixed hover Z (TABLE_Z + 150mm)
HOVER_Z_MM = -116.09 + 150.0


class Position3DNode(Node):
    def __init__(self):
        super().__init__('position_3d')
        self.bridge = CvBridge()

        # ── Load calibration ───────────────────────────────────────────────
        self.T = None
        self._load_calibration()

        # ── Load YOLO ─────────────────────────────────────────────────────
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded OK')

        # ── Point cloud cache ──────────────────────────────────────────────
        self.xyz         = None
        self.u_proj      = None
        self.v_proj      = None
        self.pc_frame_id = ''

        # ── Frame counter ──────────────────────────────────────────────────
        self.frame_count = 0

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/camera/depth/points',
            self.cloud_cb, SENSOR_QOS)

        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.color_cb, SENSOR_QOS)

        # ── Publishers ────────────────────────────────────────────────────
        self.detections_pub = self.create_publisher(
            String, '/detections', 10)

        self.img_pub = self.create_publisher(
            Image, '/yolo/detected_image', 10)

        # Backwards compatibility
        self.pos_pub = self.create_publisher(
            PointStamped, '/object/position_3d', 10)

        self.get_logger().info(
            f'\n{"="*52}\n'
            f'  Layer 3: Position3D (multi-object)\n'
            f'  Mode       : ALL classes\n'
            f'  Confidence : {CONFIDENCE_THRESHOLD}\n'
            f'  Calibration: {"LOADED" if self.T is not None else "NOT FOUND"}\n'
            f'  Publishers :\n'
            f'    /detections          (JSON String)\n'
            f'    /yolo/detected_image (annotated frame)\n'
            f'    /object/position_3d  (PointStamped, compat)\n'
            f'{"="*52}'
        )

    # ── Load calibration ───────────────────────────────────────────────────

    def _load_calibration(self):
        if not os.path.exists(CALIB_FILE):
            self.get_logger().warn(
                f'calibration.yaml not found at {CALIB_FILE}\n'
                f'robot_xy will not be published in /detections')
            return

        with open(CALIB_FILE, 'r') as f:
            data = yaml.safe_load(f)

        T_data = data['calibration']['T_camera_to_base']['data']
        self.T  = np.array(T_data, dtype=np.float64).reshape(4, 4)
        self.get_logger().info('Calibration loaded OK')

    # ── Camera → robot transform ───────────────────────────────────────────

    def _to_robot_xy(self, cam_xyz):
        """Convert camera XYZ (m) to robot base frame XY (mm)."""
        if self.T is None:
            return None, None
        cam_h   = np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0])
        robot_m = self.T @ cam_h
        x_mm    = robot_m[0] * 1000.0 + OFFSET_X_MM
        y_mm    = robot_m[1] * 1000.0 + OFFSET_Y_MM
        return round(float(x_mm), 2), round(float(y_mm), 2)

    # ── Point Cloud Callback ───────────────────────────────────────────────

    def cloud_cb(self, msg: PointCloud2):
        try:
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

            valid = (
                np.isfinite(zs) & np.isfinite(xs) & np.isfinite(ys) &
                (zs > MIN_DEPTH) & (zs < MAX_DEPTH)
            )
            if not np.any(valid):
                return

            xv = xs[valid]; yv = ys[valid]; zv = zs[valid]
            self.u_proj      = (xv * FX / zv) + CX
            self.v_proj      = (yv * FY / zv) + CY
            self.xyz         = np.stack([xv, yv, zv], axis=1)
            self.pc_frame_id = msg.header.frame_id

        except Exception as e:
            self.get_logger().warn(f'cloud_cb error: {e}')

    # ── 3D Lookup ──────────────────────────────────────────────────────────

    def get_xyz_at_pixel(self, u, v, radius=8):
        if self.xyz is None:
            return None
        dist_sq = (self.u_proj - u)**2 + (self.v_proj - v)**2
        nearby  = dist_sq < (radius ** 2)
        if not np.any(nearby):
            idx = np.argmin(dist_sq)
            pts = self.xyz[idx]
            return (float(pts[0]), float(pts[1]), float(pts[2]))
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

        self.frame_count += 1
        if self.frame_count % YOLO_SKIP != 0:
            cv2.imshow('Layer3: 3D Position', color)
            cv2.waitKey(1)
            return

        results = self.model(color, verbose=False)

        # Dict: label → latest detection data (overwrites duplicates)
        detections_dict = {}

        for box in results[0].boxes:
            conf = float(box.conf)
            if conf < CONFIDENCE_THRESHOLD:
                continue

            cls_name = self.model.names[int(box.cls)]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # ── Get 3D position ───────────────────────────────────────────
            xyz = self.get_xyz_at_pixel(cx, cy)

            if xyz is None:
                cv2.rectangle(color, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(color, f'{cls_name} NO DEPTH',
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2)
                continue

            cam_x, cam_y, cam_z = xyz

            # ── Transform to robot frame ──────────────────────────────────
            robot_x, robot_y = self._to_robot_xy(xyz)

            # ── Store detection (latest overwrites) ───────────────────────
            detections_dict[cls_name] = {
                'label'       : cls_name,
                'confidence'  : round(conf, 3),
                'camera_xyz'  : [
                    round(cam_x, 4),
                    round(cam_y, 4),
                    round(cam_z, 4)
                ],
                'robot_xy_mm' : [robot_x, robot_y]
                    if robot_x is not None else None,
                'robot_z_mm'  : round(HOVER_Z_MM, 2),
            }

            # ── Draw on frame ─────────────────────────────────────────────
            color_box = (0, 255, 0) if robot_x is not None else (0, 165, 255)
            cv2.rectangle(color, (x1, y1), (x2, y2), color_box, 2)
            cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

            if robot_x is not None:
                label = (f'{cls_name} {conf:.2f} | '
                         f'R:({robot_x:.0f},{robot_y:.0f})mm')
            else:
                label = f'{cls_name} {conf:.2f} | Z={cam_z:.2f}m'

            lsz, _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(color,
                          (x1, y1 - lsz[1] - 8),
                          (x1 + lsz[0], y1),
                          color_box, -1)
            cv2.putText(color, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

            # ── Log ───────────────────────────────────────────────────────
            self.get_logger().info(
                f'  [{cls_name}] conf={conf:.2f} | '
                f'cam=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f})m | '
                f'robot=({robot_x},{robot_y})mm'
            )

            # ── Publish PointStamped (backwards compat) ───────────────────
            pt_msg                 = PointStamped()
            pt_msg.header          = msg.header
            pt_msg.header.frame_id = self.pc_frame_id
            pt_msg.point.x         = cam_x
            pt_msg.point.y         = cam_y
            pt_msg.point.z         = cam_z
            self.pos_pub.publish(pt_msg)

        # ── Publish /detections as JSON ───────────────────────────────────
        if detections_dict:
            detections_list = list(detections_dict.values())
            json_msg      = String()
            json_msg.data = json.dumps(detections_list)
            self.detections_pub.publish(json_msg)

        # ── HUD ───────────────────────────────────────────────────────────
        n = len(detections_dict)
        hud = f'Detected: {n} object(s)'
        cv2.putText(color, hud, (10, IMG_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        if n == 0:
            cv2.putText(color, 'No objects detected',
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 165, 255), 2)

        # ── Publish annotated image ───────────────────────────────────────
        annotated_msg        = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
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
