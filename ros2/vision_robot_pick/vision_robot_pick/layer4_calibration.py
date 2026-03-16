#!/usr/bin/env python3
"""
Layer 4 - Hand-Eye Calibration (Eye-to-Hand, Point Correspondence)
Method : Move robot TCP to touch object → record (robot_xyz, camera_xyz) pairs
         Solve camera-to-base transform using SVD least squares
Object : cell phone (YOLO label = 'cell phone')
Run    : ros2 run vision_robot_pick calibration
Pass   : calibration.yaml saved with reprojection error < 10mm
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
ROBOT_IP     = '192.168.58.2'
TARGET_CLASS = 'cell phone'
MIN_POINTS   = 10
CALIB_FILE   = os.path.expanduser(
    '~/ros2_ws/src/vision_robot_pick/config/calibration.yaml')

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')

        self._devnull = open(os.devnull, 'w')

        print(f'Connecting to FAIRINO @ {ROBOT_IP} ...')
        self.robot = Robot.RPC(ROBOT_IP)
        print('Robot connected OK\n')

        # Suppress SDK spam
        sys.stdout = self._devnull

        # Storage
        self.robot_pts      = []
        self.camera_pts     = []
        self.last_robot_xyz = None

        # Latest camera XYZ — just the most recent single detection
        self.latest_cam_xyz  = None
        self.latest_cam_lock = threading.Lock()
        self.latest_cam_time = None   # timestamp of last detection

        self.create_subscription(
            PointStamped,
            '/object/position_3d',
            self.position_cb,
            SENSOR_QOS
        )

        sys.stdout = sys.__stdout__
        self._print_instructions()

        self.input_thread = threading.Thread(
            target=self.input_loop, daemon=True)
        self.input_thread.start()

    # ── Instructions ───────────────────────────────────────────────────────

    def _print_instructions(self):
        print('\n' + '='*56)
        print('  LAYER 4 - Hand-Eye Calibration')
        print('  Object  : cell phone')
        print(f'  Target  : {MIN_POINTS}+ calibration points')
        print('='*56)
        print()
        print('  WORKFLOW PER POINT:')
        print('  1. Place phone in camera view')
        print('  2. Confirm green box in Layer 3 window')
        print('  3. Enable DRAG MODE (physical button)')
        print('  4. Drag TCP to touch CENTER of phone screen')
        print('  5. Disable drag mode (robot holds position)')
        print('  6. Press ENTER to record')
        print('  7. Move phone to new position → repeat')
        print()
        print('  NOTE: Camera uses the LATEST detected XYZ.')
        print('        Detect phone BEFORE moving robot arm.')
        print('        Once detected, that XYZ is held until')
        print('        the next detection overwrites it.')
        print()
        print('  COMMANDS:')
        print('  ENTER      → collect point')
        print('  s + ENTER  → solve and save')
        print('  d + ENTER  → show collected points')
        print('  r + ENTER  → remove last point')
        print('  q + ENTER  → quit')
        print('='*56)
        print('\n  Ready.\n')

    # ── Camera callback — store LATEST single detection ────────────────────

    def position_cb(self, msg: PointStamped):
        """
        Store the most recent camera XYZ directly.
        No averaging — just the latest value.
        Also store the ROS timestamp so we can warn if detection is stale.
        """
        with self.latest_cam_lock:
            self.latest_cam_xyz  = (
                msg.point.x,
                msg.point.y,
                msg.point.z
            )
            self.latest_cam_time = self.get_clock().now()

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

            if   cmd == 's': self.solve_calibration()
            elif cmd == 'd': self.show_points()
            elif cmd == 'r': self.remove_last_point()
            elif cmd == 'q':
                print('\n  Quitting...')
                rclpy.shutdown()
                break
            else:
                self.collect_point()

            sys.stdout = self._devnull

    # ── Show points ────────────────────────────────────────────────────────

    def show_points(self):
        n = len(self.robot_pts)
        if n == 0:
            print('  No points collected yet.')
            return
        print(f'\n  ── Collected Points ({n}) ───────────────────────')
        for i, (r, c) in enumerate(zip(self.robot_pts, self.camera_pts)):
            print(f'  [{i+1:02d}] '
                  f'Robot(mm): X={r[0]:8.2f} Y={r[1]:8.2f} Z={r[2]:8.2f} | '
                  f'Cam(m): X={c[0]:6.4f} Y={c[1]:6.4f} Z={c[2]:6.4f}')
        print()

    # ── Remove last point ──────────────────────────────────────────────────

    def remove_last_point(self):
        if not self.robot_pts:
            print('  No points to remove.')
            return
        rp = self.robot_pts.pop()
        cp = self.camera_pts.pop()
        # Reset last_robot_xyz to second-to-last point
        self.last_robot_xyz = self.robot_pts[-1] if self.robot_pts else None
        print(f'  [Removed] Last point:')
        print(f'    Robot (mm): X={rp[0]:.2f} Y={rp[1]:.2f} Z={rp[2]:.2f}')
        print(f'    Cam   (m) : X={cp[0]:.4f} Y={cp[1]:.4f} Z={cp[2]:.4f}')
        print(f'  Total points remaining: {len(self.robot_pts)}')

    # ── Collect one point ──────────────────────────────────────────────────

    def collect_point(self):

        # 1. Get robot TCP
        err, tcp = self.robot.GetActualTCPPose(flag=0)
        if err != 0:
            print(f'  [ERROR] GetActualTCPPose failed, err={err}')
            return

        robot_xyz = [tcp[0], tcp[1], tcp[2]]

        # 2. Duplicate position check
        if self.last_robot_xyz is not None:
            dist = np.linalg.norm(
                np.array(robot_xyz) - np.array(self.last_robot_xyz))
            if dist < 5.0:
                print(f'\n  [WARNING] Robot barely moved! ({dist:.2f}mm)')
                print(f'            Move phone to a NEW position,')
                print(f'            drag TCP to touch it, then press ENTER.')
                return

        # 3. Get latest camera XYZ
        with self.latest_cam_lock:
            cam_xyz   = self.latest_cam_xyz
            cam_time  = self.latest_cam_time

        if cam_xyz is None:
            print('  [ERROR] No camera detection yet.')
            print('          Make sure position_3d is running')
            print('          and cell phone is/was visible.')
            return

        # 4. Warn if camera detection is stale (> 5 seconds old)
        if cam_time is not None:
            age_sec = (self.get_clock().now() - cam_time).nanoseconds / 1e9
            if age_sec > 120.0:
                print(f'\n  [WARNING] Camera XYZ is {age_sec:.1f}s old!')
                print(f'            Phone may have moved since last detection.')
                print(f'            Place phone back, confirm green box,')
                print(f'            then drag TCP and press ENTER.')
                return

        # 5. Store pair
        self.robot_pts.append(robot_xyz)
        self.camera_pts.append(list(cam_xyz))
        self.last_robot_xyz = robot_xyz

        n = len(self.robot_pts)
        print(f'\n  [Point {n:02d} collected] OK')
        print(f'    Robot  (mm) : '
              f'X={robot_xyz[0]:8.2f}  '
              f'Y={robot_xyz[1]:8.2f}  '
              f'Z={robot_xyz[2]:8.2f}')
        print(f'    Camera  (m) : '
              f'X={cam_xyz[0]:7.4f}  '
              f'Y={cam_xyz[1]:7.4f}  '
              f'Z={cam_xyz[2]:7.4f}')

        remaining = MIN_POINTS - n
        if remaining > 0:
            print(f'\n    [{n}/{MIN_POINTS}] '
                  f'{remaining} more needed. '
                  f'Move phone → drag TCP → ENTER')
        else:
            print(f'\n    [{n} points] '
                  f'Ready to solve! Type  s + ENTER')
            print(f'    Or collect more for better accuracy.')

    # ── Solve ──────────────────────────────────────────────────────────────

    def solve_calibration(self):
        n = len(self.robot_pts)
        if n < MIN_POINTS:
            print(f'  [ERROR] Need {MIN_POINTS} minimum. Have {n}.')
            return

        print(f'\n  Solving with {n} point pairs...')

        R_pts = np.array(self.robot_pts,  dtype=np.float64) / 1000.0
        C_pts = np.array(self.camera_pts, dtype=np.float64)

        # Centroids
        R_centroid = R_pts.mean(axis=0)
        C_centroid = C_pts.mean(axis=0)

        # Centre point sets
        R_c = R_pts - R_centroid
        C_c = C_pts - C_centroid

        # SVD
        H        = C_c.T @ R_c
        U, S, Vt = np.linalg.svd(H)
        R        = Vt.T @ U.T

        # Reflection fix
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        # Translation
        t = R_centroid - R @ C_centroid

        # 4x4 transform
        T      = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = t

        # Per-point errors
        errors = []
        for i in range(n):
            cam_h  = np.append(C_pts[i], 1.0)
            pred   = (T @ cam_h)[:3]
            err_mm = np.linalg.norm(pred - R_pts[i]) * 1000.0
            errors.append(err_mm)

        mean_err = np.mean(errors)
        max_err  = np.max(errors)

        print(f'\n  ── Calibration Results ──────────────────────────────')
        print(f'  Points used        : {n}')
        print(f'  Mean error         : {mean_err:.2f} mm')
        print(f'  Max  error         : {max_err:.2f} mm')
        print(f'  Rotation R         :\n{np.round(R, 5)}')
        print(f'  Translation t  (m) : {np.round(t, 5)}')

        print(f'\n  ── Per-point errors ─────────────────────────────────')
        for i, e in enumerate(errors):
            flag = '  <- HIGH' if e > 15 else ''
            print(f'    Point {i+1:02d}: {e:.2f} mm{flag}')

        print()
        if mean_err < 10.0:
            print('  [EXCELLENT] Mean error < 10mm')
        elif mean_err < 15.0:
            print('  [ACCEPTABLE] Mean error < 15mm')
        else:
            print('  [POOR] Mean error > 15mm')
            high = [i+1 for i, e in enumerate(errors) if e > 15]
            print(f'         Bad points: {high}')
            print(f'         Use  r + ENTER  to remove last point,')
            print(f'         or restart and recollect all points.')

        self.save_calibration(T, R, t, mean_err, max_err, n, errors)

    # ── Save ───────────────────────────────────────────────────────────────

    def save_calibration(self, T, R, t, mean_err, max_err, n, errors):
        os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)

        data = {
            'calibration': {
                'method'            : 'point_correspondence_svd',
                'n_points'          : int(n),
                'mean_error_mm'     : float(round(mean_err, 4)),
                'max_error_mm'      : float(round(max_err,  4)),
                'per_point_errors'  : [float(round(e, 4)) for e in errors],
                'camera_frame'      : 'camera_depth_optical_frame',
                'robot_frame'       : 'base_link',
                'T_camera_to_base'  : {
                    'data': T.flatten().tolist(),
                    'rows': 4,
                    'cols': 4,
                },
                'rotation'          : {
                    'data': R.flatten().tolist(),
                    'rows': 3,
                    'cols': 3,
                },
                'translation_meters': t.tolist(),
                'collected_points'  : {
                    'robot_mm': self.robot_pts,
                    'camera_m': self.camera_pts,
                }
            }
        }

        with open(CALIB_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        print(f'\n  [SAVED] {CALIB_FILE}')
        print(f'  Next: ros2 run vision_robot_pick transform_node\n')


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout = sys.__stdout__
        node._devnull.close()
        node.robot.CloseRPC()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
