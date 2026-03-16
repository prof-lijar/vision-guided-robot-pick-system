#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from fairino import Robot


class SDKVersionNode(Node):

    def __init__(self):
        super().__init__('sdk_version_node')

        self.get_logger().info("Connecting to robot...")

        try:
            self.robot = Robot.RPC("192.168.58.2")  # change IP if needed
            
            version = self.robot.GetSDKVersion()
            self.get_logger().info(f"SDK Version: {version}")

            self.robot.CloseRPC()
            self.get_logger().info("Connection closed successfully.")

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SDKVersionNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
