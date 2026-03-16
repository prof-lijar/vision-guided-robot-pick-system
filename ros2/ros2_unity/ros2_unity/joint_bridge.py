import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from fairino_msgs.msg import RobotNonrtState
import math


class JointBridge(Node):

    def __init__(self):
        super().__init__('joint_bridge')

        self.publisher = self.create_publisher(
            JointState,
            '/fr5_joint_states',
            10
        )

        self.subscription = self.create_subscription(
            RobotNonrtState,
            '/nonrt_state_data',
            self.callback,
            10
        )

    def callback(self, msg):

        joint_msg = JointState()

        joint_msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        # Convert degrees → radians (Unity works better with radians internally)
        joint_msg.position = [
            math.radians(msg.j1_cur_pos),
            math.radians(msg.j2_cur_pos),
            math.radians(msg.j3_cur_pos),
            math.radians(msg.j4_cur_pos),
            math.radians(msg.j5_cur_pos),
            math.radians(msg.j6_cur_pos),
        ]

        self.publisher.publish(joint_msg)


def main():
    rclpy.init()
    node = JointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
