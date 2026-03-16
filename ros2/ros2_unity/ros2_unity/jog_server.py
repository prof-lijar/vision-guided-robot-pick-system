#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

import ros2_unity.robot_connection as rc

JOG_REF = {
    0: (0, 1),
    1: (2, 3),
    2: (4, 5),
}


class JogServer(Node):
    def __init__(self):
        super().__init__('jog_server')
        self.declare_parameter('default_vel',     20.0)
        self.declare_parameter('default_max_dis', 30.0)

        self.default_vel     = self.get_parameter('default_vel').value
        self.default_max_dis = self.get_parameter('default_max_dis').value

        rc.init()

        self.robot = rc.get_robot()   # shared connection
        self.create_subscription(String, '/fr_jog_cmd', self._on_jog, 10)
        self.get_logger().info('Jog server ready on /fr_jog_cmd')

    def _on_jog(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON error: {e}')
            return

        mode      = int(cmd.get('mode', 0))
        axis      = int(cmd.get('axis', 1))
        direction = int(cmd.get('direction', 0))
        vel       = float(cmd.get('velocity',  self.default_vel))
        max_dis   = float(cmd.get('max_dis',   self.default_max_dis))

        if mode not in JOG_REF:
            return

        start_ref, _ = JOG_REF[mode]

        try:
            if direction == 0:
                err = self.robot.ImmStopJOG()
                self.get_logger().debug(f'ImmStopJOG -> {err}')
            elif direction in (1, -1):
                sdk_dir = 1 if direction == 1 else 0
                err = self.robot.StartJOG(start_ref, axis, sdk_dir, max_dis, vel)
                self.get_logger().info(
                    f'StartJOG(ref={start_ref} axis={axis} dir={sdk_dir} '
                    f'max_dis={max_dis} vel={vel}) -> {err}')
        except Exception as e:
            self.get_logger().error(f'JOG error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JogServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
