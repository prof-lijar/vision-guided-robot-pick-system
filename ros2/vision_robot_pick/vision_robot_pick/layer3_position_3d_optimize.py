#!/usr/bin/env python3

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

CONFIDENCE_THRESHOLD = 0.5
MODEL_PATH = "yolov8n.pt"

# Run YOLO every N frames
YOLO_SKIP = 3

# Camera intrinsics
FX = 357.99
FY = 357.99
CX = 318.69
CY = 177.58

MIN_DEPTH = 0.1
MAX_DEPTH = 5.0


class Position3DNode(Node):

    def __init__(self):
        super().__init__("position_3d")

        self.bridge = CvBridge()

        # Cached point cloud
        self.xyz = None
        self.u_proj = None
        self.v_proj = None
        self.pc_frame_id = None

        # Frame counter
        self.frame_count = 0

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded")

        self.create_subscription(
            PointCloud2,
            "/camera/depth/points",
            self.cloud_cb,
            SENSOR_QOS
        )

        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.color_cb,
            SENSOR_QOS
        )

        self.pos_pub = self.create_publisher(
            PointStamped,
            "/object/position_3d",
            10
        )

        self.get_logger().info("Layer3 optimized node started")

    # ---------------------------------
    # Point Cloud Callback
    # ---------------------------------

    def cloud_cb(self, msg):

        try:
            pts = np.array([
                [p[0], p[1], p[2]]
                for p in pc2.read_points(
                    msg,
                    field_names=["x", "y", "z"],
                    skip_nans=False
                )
            ], dtype=np.float32)

            if pts.size == 0:
                return

            z = pts[:, 2]

            valid = (
                np.isfinite(z) &
                (z > MIN_DEPTH) &
                (z < MAX_DEPTH)
            )

            pts = pts[valid]

            if len(pts) == 0:
                return

            x = pts[:, 0]
            y = pts[:, 1]
            z = pts[:, 2]

            u_proj = (x * FX / z) + CX
            v_proj = (y * FY / z) + CY

            self.xyz = pts
            self.u_proj = u_proj
            self.v_proj = v_proj
            self.pc_frame_id = msg.header.frame_id

        except Exception as e:
            self.get_logger().warn(f"Point cloud processing error: {e}")

    # ---------------------------------
    # Find 3D point near pixel
    # ---------------------------------

    def get_xyz_at_pixel(self, u, v, radius=8):

        if self.xyz is None:
            return None

        dist_sq = (self.u_proj - u) ** 2 + (self.v_proj - v) ** 2

        nearby = dist_sq < radius ** 2

        if not np.any(nearby):
            return None

        pts = self.xyz[nearby]

        return (
            float(np.median(pts[:, 0])),
            float(np.median(pts[:, 1])),
            float(np.median(pts[:, 2]))
        )

    # ---------------------------------
    # Color Image Callback
    # ---------------------------------

    def color_cb(self, msg):

        if self.xyz is None:
            return

        color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Increment frame counter
        self.frame_count += 1

        # Skip YOLO frames
        if self.frame_count % YOLO_SKIP != 0:

            cv2.imshow("Layer3 3D Position", color)
            cv2.waitKey(1)
            return

        results = self.model(color, verbose=False)

        for box in results[0].boxes:

            conf = float(box.conf)

            if conf < CONFIDENCE_THRESHOLD:
                continue

            cls_name = self.model.names[int(box.cls)]

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            xyz = self.get_xyz_at_pixel(cx, cy)

            if xyz is None:

                self.get_logger().warn(
                    f"{cls_name} | pixel({cx},{cy}) | no depth"
                )
                continue

            x, y, z = xyz

            self.get_logger().info(
                f"{cls_name} | X={x:.3f} Y={y:.3f} Z={z:.3f}"
            )

            pt = PointStamped()
            pt.header = msg.header
            pt.header.frame_id = self.pc_frame_id

            pt.point.x = x
            pt.point.y = y
            pt.point.z = z

            self.pos_pub.publish(pt)

            # Draw bounding box
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

            # Create label with distance
            label = f"{cls_name} {z:.2f}m"

            # Draw label background
            (tw, th), _ = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                2
            )

            cv2.rectangle(
                color,
                (x1, y1 - th - 6),
                (x1 + tw, y1),
                (0, 255, 0),
                -1
            )

            # Draw label text
            cv2.putText(
                color,
                label,
                (x1, y1 - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2
            )

        cv2.imshow("Layer3 3D Position", color)
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


if __name__ == "__main__":
    main()
