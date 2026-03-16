#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import ros2_unity.robot_connection as rc


class CmdServer(Node):
    def __init__(self):
        super().__init__('cmd_server')

        self.robot = rc.get_robot()   # shared connection
        self.create_subscription(String, '/fr_robot_cmd', self._on_cmd, 10)
        self.result_pub = self.create_publisher(String, '/fr_robot_cmd_result', 10)
        self.get_logger().info('Cmd server ready on /fr_robot_cmd')

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'CMD: {cmd}')
        result = self._dispatch(cmd)
        out = String()
        out.data = f'{cmd} -> {result}'
        self.result_pub.publish(out)

    def _dispatch(self, cmd: str) -> str:
        match = re.fullmatch(r'(\w+)\(([^)]*)\)', cmd)
        if not match:
            return f'PARSE_ERROR: {cmd}'
        func = match.group(1)
        raw  = match.group(2).strip()
        args = [a.strip() for a in raw.split(',')] if raw else []
        try:
            return self._run(func, args)
        except Exception as e:
            return f'EXCEPTION: {e}'

    def _run(self, func: str, args: list) -> str:
        r = self.robot
        def f(i): return float(args[i])
        def iv(i): return int(float(args[i]))

        if   func == 'RobotEnable':     return str(r.RobotEnable(iv(0)))
        elif func == 'Mode':            return str(r.Mode(iv(0)))
        elif func == 'SetSpeed':        return str(r.SetSpeed(iv(0)))
        elif func == 'ResetAllError':   return str(r.ResetAllError())
        elif func == 'ImmStopJOG':      return str(r.ImmStopJOG())
        elif func == 'StopMotion':      return str(r.StopMotion())
        elif func == 'DragTeachSwitch': return str(r.DragTeachSwitch(state=iv(0)))
        elif func == 'MoveJ':
            return str(r.MoveJ(joint_pos=[f(i) for i in range(6)],
                               tool=iv(6), user=iv(7), vel=f(8)))
        elif func == 'MoveL':
            return str(r.MoveL(desc_pos=[f(i) for i in range(6)],
                               tool=iv(6), user=iv(7), vel=f(8)))
        elif func == 'GetActualJointPosDegree':
            err, pos = r.GetActualJointPosDegree(0)
            return f'{err},' + ','.join(str(round(p, 3)) for p in pos)
        elif func == 'GetActualTCPPose':
            err, pos = r.GetActualTCPPose(0)
            return f'{err},' + ','.join(str(round(p, 3)) for p in pos)
        elif func == 'GetRobotState':
            err, state = r.GetRobotState()
            return f'{err},{state}'
        else:
            return f'UNKNOWN: {func}'
    
    def _run_demo_motion(self):
        r = self.robot

        j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
        j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

        desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
        desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
        desc_pos3 = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
        desc_pos4 = [-443.165, 147.881, 480.951, 179.511, -0.775, -15.409]

        tool = 0
        user = 0
        vel = 50.0
        blendT = 0.0
        blendR = 0.0

        self.get_logger().info("Starting demo motion sequence...")

        r.SetSpeed(20)

        err = r.SingularAvoidStart(2, 10, 5, 5)
        self.get_logger().info(f"SingularAvoidStart -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, blendT=blendT)
        self.get_logger().info(f"MoveJ j1 -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, blendR=blendR)
        self.get_logger().info(f"MoveL -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.MoveC(
            desc_pos_p=desc_pos3, tool_p=tool, user_p=user,
            desc_pos_t=desc_pos4, tool_t=tool, user_t=user,
            blendR=blendR
        )
        self.get_logger().info(f"MoveC -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, blendT=blendT)
        self.get_logger().info(f"MoveJ j2 -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.Circle(
            desc_pos_p=desc_pos3, tool_p=tool, user_p=user,
            desc_pos_t=desc_pos1, tool_t=tool, user_t=user
        )
        self.get_logger().info(f"Circle -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
        self.get_logger().info(f"MoveCart -> {err}")
        if err != 0: return f"Error: {err}"

        err = r.SingularAvoidEnd()
        self.get_logger().info(f"SingularAvoidEnd -> {err}")

        return "DemoMotion Finished"

def main(args=None):
    rclpy.init(args=args)
    rc.init()
    node = CmdServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
