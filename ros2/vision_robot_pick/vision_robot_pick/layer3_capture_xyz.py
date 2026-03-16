#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import numpy as np
import cv2

from ultralytics import YOLO
import message_filters


class ObjectXYZ(Node):

    def __init__(self):

        super().__init__("layer3_capture_xyz")

        self.bridge = CvBridge()

        self.rgb = None
        self.depth = None
        self.info = None

        self.model = YOLO("yolov8n.pt")

        rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw"
        )

        depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/depth/image_raw"
        )

        info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            "/camera/color/camera_info"
        )

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.1
        )

        sync.registerCallback(self.callback)

        self.get_logger().info("Layer3 ready (cell phone detection)")

    def callback(self, rgb_msg, depth_msg, info_msg):

        self.rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.depth = self.bridge.imgmsg_to_cv2(depth_msg)
        self.info = info_msg

    def pixel_to_xyz(self, u, v, z):

        fx = self.info.k[0]
        fy = self.info.k[4]
        cx = self.info.k[2]
        cy = self.info.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return x, y, z

    def capture(self):

        if self.rgb is None:
            print("Waiting for camera...")
            return None

        results = self.model(self.rgb)[0]

        boxes = results.boxes
        names = results.names

        target_box = None

        for box in boxes:

            cls_id = int(box.cls[0])
            label = names[cls_id]

            if label == "cell phone":
                target_box = box
                break

        if target_box is None:
            print("No cell phone detected")
            return None

        x1, y1, x2, y2 = map(int, target_box.xyxy[0].cpu().numpy())

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        depths = []

        for _ in range(30):

            u = np.random.randint(x1, x2)
            v = np.random.randint(y1, y2)

            z = self.depth[v, u]

            if z > 0:
                depths.append(z)

        if len(depths) == 0:
            print("No valid depth")
            return None

        z = np.median(depths) / 1000.0   # mm → meters

        x, y, z = self.pixel_to_xyz(cx, cy, z)

        print("\nCamera XYZ (meters)")
        print(x, y, z)

        vis = self.rgb.copy()

        cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.circle(vis, (cx, cy), 5, (0,0,255), -1)

        label = f"{x:.3f},{y:.3f},{z:.3f}"

        cv2.putText(
            vis,
            label,
            (x1, y1-10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0,255,0),
            2
        )

        cv2.imshow("Cell Phone Detection", vis)
        cv2.waitKey(1)

        return np.array([x, y, z])


def main():

    rclpy.init()

    node = ObjectXYZ()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    import threading
    threading.Thread(target=executor.spin, daemon=True).start()

    while True:

        input("\nPress ENTER to capture cell phone position")

        p = node.capture()

        if p is not None:
            print("Camera point:", p)


if __name__ == "__main__":
    main()
