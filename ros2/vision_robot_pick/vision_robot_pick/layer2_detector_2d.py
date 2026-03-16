#!/usr/bin/env python3
"""
Layer 2 - YOLO 2D Detection Node
Tests : detect objects in color image, draw bounding boxes
Topics: subscribes /camera/color/image_raw
        publishes  /detections/image_raw (visualization)
Run   : ros2 run vision_robot_pick detector_2d
Pass  : bounding boxes drawn on screen with class name + confidence
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)

# Detection settings
CONFIDENCE_THRESHOLD = 0.3
MODEL_PATH           = 'yolov8n.pt'   # downloads automatically on first run


class Detector2DNode(Node):
    def __init__(self):
        super().__init__('detector_2d')

        self.bridge = CvBridge()

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {MODEL_PATH} ...')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded OK')

        # Subscriber
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.color_cb, SENSOR_QOS)

        # Publisher - annotated image for debugging
        self.vis_pub = self.create_publisher(
            Image, '/detections/image_raw', 10)

        # Stats
        self.frame_count    = 0
        self.detect_count   = 0

        self.get_logger().info(
            'Layer 2: Detector2D started\n'
            f'  model      : {MODEL_PATH}\n'
            f'  confidence : {CONFIDENCE_THRESHOLD}\n'
            '  waiting for color frames...'
        )

    def color_cb(self, msg: Image):
        self.frame_count += 1
        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO inference
        results = self.model(color, verbose=False)
        detections = results[0].boxes

        frame_detections = 0

        for box in detections:
            conf = float(box.conf)
            if conf < CONFIDENCE_THRESHOLD:
                continue

            frame_detections += 1
            self.detect_count += 1

            cls_id   = int(box.cls)
            cls_name = self.model.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Center point
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Draw bounding box
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

            # Label: class + confidence
            label = f'{cls_name} {conf:.2f}'
            label_size, _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)

            # Label background
            cv2.rectangle(
                color,
                (x1, y1 - label_size[1] - 10),
                (x1 + label_size[0], y1),
                (0, 255, 0), -1)

            # Label text
            cv2.putText(
                color, label, (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            # Log each detection
            self.get_logger().info(
                f'  [{cls_name}] conf={conf:.2f} | '
                f'box=({x1},{y1})-({x2},{y2}) | '
                f'center=({cx},{cy})'
            )

        # HUD overlay - frame counter + detection count
        cv2.putText(
            color,
            f'Frame: {self.frame_count}  Detections: {frame_detections}',
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Show window
        cv2.imshow('Layer2: YOLO 2D Detection', color)
        cv2.waitKey(1)

        # Publish annotated image
        self.vis_pub.publish(
            self.bridge.cv2_to_imgmsg(color, 'bgr8'))

        # Summary every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'[Stats] frames={self.frame_count} | '
                f'total_detections={self.detect_count}'
            )


def main():
    rclpy.init()
    node = Detector2DNode()
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
