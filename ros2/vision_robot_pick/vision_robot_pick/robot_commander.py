#!/usr/bin/env python3
"""
Layer 6 - Robot Commander
Subscribes to /detections (JSON from Layer 3) and /robot_command (String).
When a command arrives, looks up the named object in current detections
and moves the robot above it.

Topics subscribed:
  /detections    std_msgs/String  — JSON array from Layer 3
  /robot_command std_msgs/String  — object label to move to
                                    e.g. "cell phone", "car", "bottle"

Run:
  ros2 run vision_robot_pick commander

Send command:
  ros2 topic pub /robot_command std_msgs/String "data: 'cell phone'" --once
  ros2 topic pub /robot_command std_msgs/String "data: 'car'" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import numpy as np
import json
import sys
import os
import threading
from fairino import Robot

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_IP    = '192.168.58.2'
HOVER_Z_MM  = -116.09 + 150.0    # TABLE_Z + 150mm
MOVE_VEL    = 10                  # % speed

# Fixed straight-down TCP orientation (measured)
TCP_RX = -177.293
TCP_RY =   -2.013
TCP_RZ = -121.047

# Safe home joint position (measured)
HOME_JOINTS = [128.724, -47.142, 105.43, -151.582, -89.579, -43.755]

# Detection timeout — ignore detections older than this
DETECTION_TIMEOUT_SEC = 5.0

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)


class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander')

        self._devnull = open(os.devnull, 'w')

        # ── Connect robot ──────────────────────────────────────────────────
        print(f'\n  Connecting to FAIRINO @ {ROBOT_IP} ...')
        self.robot = Robot.RPC(ROBOT_IP)
        sys.stdout = self._devnull
        sys.stdout = sys.__stdout__
        print('  Robot connected OK\n')

        # ── State ──────────────────────────────────────────────────────────
        # Dict: label → {label, robot_xy_mm, robot_z_mm, timestamp}
        self.detections      = {}
        self.detections_lock = threading.Lock()
        self.moving          = False

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            String, '/detections',
            self.detections_cb, 10)

        self.create_subscription(
            String, '/robot_command',
            self.command_cb, SENSOR_QOS)

        self._print_instructions()

        self.create_subscription(String, '/robot_speed', self.speed_cb, 10)
        self.current_speed = 20  # default

    # ── Instructions ───────────────────────────────────────────────────────

    def _print_instructions(self):
        print(f'{"="*56}')
        print(f'  LAYER 6 - Robot Commander')
        print(f'  Hover Z    : {HOVER_Z_MM:.2f}mm  (table + 150mm)')
        print(f'  Speed      : {MOVE_VEL}%')
        print(f'{"="*56}')
        print()
        print('  Waiting for commands on /robot_command ...')
        print()
        print('  Send command examples:')
        print('  ros2 topic pub /robot_command std_msgs/String \\')
        print('    "data: \'cell phone\'" --once')
        print()
        print('  ros2 topic pub /robot_command std_msgs/String \\')
        print('    "data: \'car\'" --once')
        print()
        print('  Or list current detections:')
        print('  ros2 topic echo /detections --once')
        print(f'{"="*56}\n')

    # ── Detections callback ────────────────────────────────────────────────

    def detections_cb(self, msg: String):
        """Parse JSON detections from Layer 3 and store latest per label."""
        try:
            detections_list = json.loads(msg.data)
            now = self.get_clock().now()

            with self.detections_lock:
                for det in detections_list:
                    label = det.get('label')
                    if not label:
                        continue
                    # Only store if robot_xy is available (calibration loaded)
                    if det.get('robot_xy_mm') is None:
                        continue
                    self.detections[label] = {
                        'label'      : label,
                        'robot_xy_mm': det['robot_xy_mm'],   # [x_mm, y_mm]
                        'robot_z_mm' : det.get('robot_z_mm', HOVER_Z_MM),
                        'camera_xyz' : det.get('camera_xyz'),
                        'confidence' : det.get('confidence'),
                        'timestamp'  : now,
                    }

        except Exception as e:
            self.get_logger().warn(f'detections_cb parse error: {e}')

    # ── Command callback ───────────────────────────────────────────────────

    def command_cb(self, msg: String):
        """Receive object label command and move robot above it."""
        target_label = msg.data.strip()

        if not target_label:
            self.get_logger().warn('Empty command received, ignoring.')
            return

        print(f'\n  [COMMAND] → "{target_label}"')

        if self.moving:
            print(f'  [BUSY] Robot is still moving. Command ignored.')
            return

        # ── Look up detection ──────────────────────────────────────────────
        with self.detections_lock:
            det = self.detections.get(target_label)

        if det is None:
            print(f'  [ERROR] "{target_label}" not currently detected.')
            print(f'  Currently detected objects:')
            with self.detections_lock:
                if self.detections:
                    for lbl in self.detections:
                        print(f'    - {lbl}')
                else:
                    print(f'    (none)')
            return

        # ── Stale check ────────────────────────────────────────────────────
        age_sec = (self.get_clock().now() - det['timestamp']).nanoseconds / 1e9
        if age_sec > DETECTION_TIMEOUT_SEC:
            print(f'  [WARNING] Detection of "{target_label}" is '
                  f'{age_sec:.1f}s old (max {DETECTION_TIMEOUT_SEC}s).')
            print(f'            Make sure object is visible in camera.')
            return

        # ── Build target pose ──────────────────────────────────────────────
        x_mm, y_mm = det['robot_xy_mm']
        target = [x_mm, y_mm, HOVER_Z_MM, TCP_RX, TCP_RY, TCP_RZ]

        print(f'  Object     : {target_label}  '
              f'(conf={det["confidence"]:.2f}, age={age_sec:.1f}s)')
        print(f'  Robot XY   : X={x_mm:.2f}mm  Y={y_mm:.2f}mm')
        print(f'  Hover Z    : {HOVER_Z_MM:.2f}mm')
        print(f'  Moving...')

        # ── Execute move ───────────────────────────────────────────────────
        threading.Thread(
            target=self._execute_move,
            args=(target, target_label),
            daemon=True
        ).start()

    # ── Execute move (in thread) ───────────────────────────────────────────

    def _execute_move(self, target, label):
        self.moving = True
        self._reset_errors()

        sys.stdout = self._devnull
        err = self.robot.MoveCart(target, 0, 0, self.current_speed)
        sys.stdout = sys.__stdout__

        if err != 0:
            print(f'  [WARNING] Direct MoveCart failed (err={err})')
            print(f'            Recovering via safe home joints...')

            self._reset_errors()
            sys.stdout = self._devnull
            err2 = self.robot.MoveJ(HOME_JOINTS, 0, 0)
            sys.stdout = sys.__stdout__

            if err2 != 0:
                self.moving = False
                print(f'  [ERROR] MoveJ to home failed: err={err2}')
                return

            print(f'  Home reached. Retrying MoveCart...')
            self._reset_errors()
            sys.stdout = self._devnull
            err3 = self.robot.MoveCart(target, 0, 0, MOVE_VEL)
            sys.stdout = sys.__stdout__

            if err3 != 0:
                self.moving = False
                print(f'  [ERROR] MoveCart failed even after home: err={err3}')
                print(f'          Target = {target}')
                return

            print(f'  Recovered OK.')

        self.moving = False

        # ── Read actual TCP after move ─────────────────────────────────────
        err2, tcp = self.robot.GetActualTCPPose(flag=0)
        if err2 == 0:
            print(f'\n  ── Result ───────────────────────────────────────')
            print(f'  Moved above  : {label}')
            print(f'  Actual TCP   : '
                  f'X={tcp[0]:.2f}mm  Y={tcp[1]:.2f}mm  Z={tcp[2]:.2f}mm')
            print(f'  Target XY    : X={target[0]:.2f}mm  Y={target[1]:.2f}mm')
            print(f'  XY residual  : '
                  f'dX={tcp[0]-target[0]:.2f}mm  '
                  f'dY={tcp[1]-target[1]:.2f}mm')
            print(f'  ─────────────────────────────────────────────────\n')

    # ── Reset robot errors ─────────────────────────────────────────────────

    def _reset_errors(self):
        sys.stdout = self._devnull
        self.robot.ResetAllError()
        sys.stdout = sys.__stdout__
    
    def speed_cb(self, msg):
        try:
            self.current_speed = int(msg.data)
            self.get_logger().info(f'Speed updated: {self.current_speed}%')
        except:
            pass

def main():
    rclpy.init()
    node = CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout = sys.__stdout__
        node._devnull.close()
        node.robot.CloseRPC()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
