#!/usr/bin/env python3
"""
Layer 5+7 Test - Calibration Verification
Loads calibration.yaml, transforms camera XYZ to robot frame,
moves robot above detected object when ENTER is pressed.

Run : ros2 run vision_robot_pick calib_test
Pass: Robot moves to correct XY position above object
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
import numpy as np
import yaml
import os
import sys
import threading
from fairino import Robot

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_IP    = '192.168.58.2'
HOVER_Z_MM  = -116.09 + 150.0    # TABLE_Z + 150mm = 33.91mm above table
MOVE_VEL    = 10 # % speed — slow and safe for testing
OFFSET_X_MM = 6.963
OFFSET_Y_MM = 38.277
# Safe home joint position (measured)
HOME_JOINTS = [128.724, -47.142, 105.43, -151.582, -89.579, -43.755]

CALIB_FILE  = os.path.expanduser(
    '~/ros2_ws/src/vision_robot_pick/config/calibration.yaml')

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)


class CalibTestNode(Node):
    def __init__(self):
        super().__init__('calib_test')

        self._devnull = open(os.devnull, 'w')

        # ── Load calibration ───────────────────────────────────────────────
        print(f'\n  Loading calibration from:')
        print(f'  {CALIB_FILE}')
        self._load_calibration()

        # ── Connect robot ──────────────────────────────────────────────────
        print(f'\n  Connecting to FAIRINO @ {ROBOT_IP} ...')
        self.robot = Robot.RPC(ROBOT_IP)
        sys.stdout = self._devnull
        sys.stdout = sys.__stdout__
        print('  Robot connected OK')

        # ── Latest camera detection ────────────────────────────────────────
        self.latest_cam_xyz  = None
        self.latest_cam_lock = threading.Lock()
        self.latest_cam_time = None
        self.moving          = False

        # ── Subscribe to Layer 3 output ────────────────────────────────────
        self.create_subscription(
            PointStamped,
            '/object/position_3d',
            self.position_cb,
            SENSOR_QOS
        )

        self._print_instructions()

        # ── Input thread ───────────────────────────────────────────────────
        self.input_thread = threading.Thread(
            target=self.input_loop, daemon=True)
        self.input_thread.start()

    # ── Load calibration ───────────────────────────────────────────────────

    def _load_calibration(self):
        if not os.path.exists(CALIB_FILE):
            raise FileNotFoundError(
                f'calibration.yaml not found at {CALIB_FILE}\n'
                f'Run calibration node first.')

        with open(CALIB_FILE, 'r') as f:
            data = yaml.safe_load(f)

        calib = data['calibration']

        # Load 4x4 transform matrix
        T_data = calib['T_camera_to_base']['data']
        self.T  = np.array(T_data, dtype=np.float64).reshape(4, 4)

        # Print calibration summary
        print(f'\n  ── Calibration Summary ──────────────────────────')
        print(f'  Method     : {calib.get("method", "unknown")}')
        print(f'  Points     : {calib.get("n_points", "?")}')

        # Support both error key names
        err = calib.get('mean_xy_error_mm') or calib.get('mean_error_mm', '?')
        print(f'  Mean error : {err} mm')
        print(f'  Transform T:')
        print(f'{np.round(self.T, 4)}')
        print(f'  ────────────────────────────────────────────────')

    # ── Camera callback ────────────────────────────────────────────────────

    def position_cb(self, msg: PointStamped):
        with self.latest_cam_lock:
            self.latest_cam_xyz  = (
                msg.point.x,
                msg.point.y,
                msg.point.z
            )
            self.latest_cam_time = self.get_clock().now()

    # ── Transform camera → robot ───────────────────────────────────────────

    def transform_to_robot(self, cam_xyz):
        """
        Apply calibration transform + correction offset
        to convert camera XYZ to robot base frame.
        Returns (x_mm, y_mm) in robot base frame.
        """
        cam_h   = np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0])
        robot_m = self.T @ cam_h
        x_mm    = robot_m[0] * 1000.0 + OFFSET_X_MM
        y_mm    = robot_m[1] * 1000.0 + OFFSET_Y_MM
        
        return x_mm, y_mm

    # ── Instructions ───────────────────────────────────────────────────────

    def _print_instructions(self):
        print(f'\n{"="*56}')
        print(f'  CALIBRATION TEST NODE')
        print(f'  Hover height : TABLE_Z + 150mm = {HOVER_Z_MM:.2f}mm')
        print(f'  Move speed   : {MOVE_VEL}% (slow/safe)')
        print(f'{"="*56}')
        print()
        print('  PROCEDURE:')
        print('  1. Make sure Layer 3 (position_3d) is running')
        print('  2. Place object in camera view')
        print('  3. Confirm green box detected in Layer 3 window')
        print('  4. Press ENTER → robot moves above object')
        print('  5. Measure how far off the robot is from object')
        print('  6. Move object to new position → repeat')
        print()
        print('  COMMANDS:')
        print('  ENTER      → move robot above detected object')
        print('  p + ENTER  → print current TCP pose only (no move)')
        print('  h + ENTER  → move robot to safe home position')
        print('  q + ENTER  → quit')
        print(f'{"="*56}')
        print('\n  Ready. Place object and press ENTER.\n')

    # ── Input loop ─────────────────────────────────────────────────────────

    def input_loop(self):
        while rclpy.ok():
            try:
                sys.stdout = sys.__stdout__
                user = input()
                sys.stdout = self._devnull
            except EOFError:
                break

            sys.stdout = sys.__stdout__
            cmd = user.strip().lower()

            if   cmd == 'q':
                print('\n  Quitting...')
                rclpy.shutdown()
                break
            elif cmd == 'p':
                self.print_tcp_pose()
            elif cmd == 'h':
                self.move_home()
            else:
                self.move_to_object()

            sys.stdout = self._devnull

    # ── Print TCP pose ─────────────────────────────────────────────────────

    def print_tcp_pose(self):
        err, tcp = self.robot.GetActualTCPPose(flag=0)
        if err != 0:
            print(f'  [ERROR] GetActualTCPPose failed: {err}')
            return
        print(f'\n  Current TCP pose:')
        print(f'    X  = {tcp[0]:8.2f} mm')
        print(f'    Y  = {tcp[1]:8.2f} mm')
        print(f'    Z  = {tcp[2]:8.2f} mm')
        print(f'    Rx = {tcp[3]:8.2f} deg')
        print(f'    Ry = {tcp[4]:8.2f} deg')
        print(f'    Rz = {tcp[5]:8.2f} deg')

    # ── Move to safe home ──────────────────────────────────────────────────

    def move_home(self):
        print('\n  Moving to home position...')
        # Read current pose — keep orientation, just raise Z
        err, tcp = self.robot.GetActualTCPPose(flag=0)
        if err != 0:
            print(f'  [ERROR] GetActualTCPPose failed: {err}')
            return

        home = [tcp[0], tcp[1], HOVER_Z_MM + 100.0,
                tcp[3], tcp[4], tcp[5]]

        sys.stdout = self._devnull
        err = self.robot.MoveL(
            dec_pos=home, tool=0, user=0, vel=MOVE_VEL)
        sys.stdout = sys.__stdout__

        if err != 0:
            print(f'  [ERROR] MoveL failed: {err}')
        else:
            print(f'  Raised Z to {home[2]:.1f}mm')

    # ── Move robot above detected object ───────────────────────────────────

    def move_to_object(self):
        if self.moving:
            print('  [BUSY] Robot is still moving...')
            return

        # 1. Get latest camera detection
        with self.latest_cam_lock:
            cam_xyz  = self.latest_cam_xyz
            cam_time = self.latest_cam_time

        if cam_xyz is None:
            print('  [ERROR] No camera detection.')
            print('          Is position_3d running?')
            print('          Is object visible in camera?')
            return

        # 2. Stale check — 30 seconds
        if cam_time is not None:
            age_sec = (self.get_clock().now() - cam_time).nanoseconds / 1e9
            if age_sec > 30.0:
                print(f'  [WARNING] Camera detection is {age_sec:.1f}s old!')
                print(f'            Place object back in camera view first.')
                return
            print(f'  Camera detection age: {age_sec:.1f}s')

        # 3. Transform camera → robot frame
        x_mm, y_mm = self.transform_to_robot(cam_xyz)

        # 4. Get current TCP orientation — keep it
        err, tcp = self.robot.GetActualTCPPose(flag=0)
        if err != 0:
            print(f'  [ERROR] GetActualTCPPose failed: {err}')
            return

        # uses current wrist orientation (causes tilt)
        # rx, ry, rz = tcp[3], tcp[4], tcp[5]

        # uses fixed straight-down orientation
        rx, ry, rz = -177.293, -2.013, -121.047

        target = [x_mm, y_mm, HOVER_Z_MM, rx, ry, rz]

        # 5. Print prediction before moving
        print(f'\n  ── Moving to object ─────────────────────────────')
        print(f'  Camera XYZ   : '
              f'X={cam_xyz[0]:.4f}m  '
              f'Y={cam_xyz[1]:.4f}m  '
              f'Z={cam_xyz[2]:.4f}m')
        print(f'  Predicted XY : X={x_mm:.2f}mm  Y={y_mm:.2f}mm')
        print(f'  Hover Z      : {HOVER_Z_MM:.2f}mm  (table + 150mm)')
        print(f'  Speed        : {MOVE_VEL}%')
        print(f'  Moving...')

        # 6. Execute move - with MoveJ fallback on failure
        self.moving = True
        self._reset_errors()
        sys.stdout = self._devnull
        err = self.robot.MoveL(desc_pos=target, tool=0, user=0, vel=MOVE_VEL)
        sys.stdout = sys.__stdout__

        if err != 0:
            print(f'  [WARNING] Direct MoveL failed (err={err})')
            print(f'            Recovering via safe home joints...')
            
            self._reset_errors()
            sys.stdout = self._devnull
            err2 = self.robot.MoveJ(HOME_JOINTS, 0, 0)
            sys.stdout = sys.__stdout__

            if err2 != 0:
                self.moving = False
                print(f'  [ERROR] MoveJ to home failed: err={err2}')
                return

            print(f'  Home reached. Retrying MoveL to target...')
            sys.stdout = self._devnull
            err3 = self.robot.MoveL(desc_pos=target, tool=0, user=0, vel=MOVE_VEL)
            sys.stdout = sys.__stdout__

            if err3 != 0:
                self.moving = False
                print(f'  [ERROR] MoveL failed even after home: err={err3}')
                print(f'          Target = {target}')
                return

            print(f'  Recovered OK.')

        self.moving = False

        # 7. Read actual TCP after move
        err2, tcp_after = self.robot.GetActualTCPPose(flag=0)
        if err2 == 0:
            print(f'\n  ── Result ───────────────────────────────────────')
            print(f'  Actual TCP   : '
                  f'X={tcp_after[0]:.2f}mm  '
                  f'Y={tcp_after[1]:.2f}mm  '
                  f'Z={tcp_after[2]:.2f}mm')
            print(f'  Predicted    : '
                  f'X={x_mm:.2f}mm  '
                  f'Y={y_mm:.2f}mm')
            print(f'  XY error     : '
                  f'dX={tcp_after[0]-x_mm:.2f}mm  '
                  f'dY={tcp_after[1]-y_mm:.2f}mm')
            print()
            print(f'  Now look at where robot TCP is vs object center.')
            print(f'  Measure offset in X and Y with ruler if needed.')
            print(f'  Move object to new position → press ENTER again.')

    def _reset_errors(self):
        """Clear any active robot alarms before motion."""
        sys.stdout = self._devnull
        err = self.robot.ResetAllError()
        sys.stdout = sys.__stdout__
        if err != 0:
            print(f'  [WARNING] ResetAllError returned: {err}')
        else:
            print(f'  [OK] Robot errors cleared.')

def main():
    rclpy.init()
    node = CalibTestNode()
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
