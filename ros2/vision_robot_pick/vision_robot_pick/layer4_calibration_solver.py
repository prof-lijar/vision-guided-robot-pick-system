#!/usr/bin/env python3

import numpy as np
from vision_robot_pick.layer3_capture_xyz import ObjectXYZ

from fairino import Robot
import rclpy


ROBOT_IP = "192.168.58.2"


def get_robot_xyz(robot):

    pose = robot.GetActualTCPPose()

    x = pose[0] / 1000.0
    y = pose[1] / 1000.0
    z = pose[2] / 1000.0

    return np.array([x,y,z])


def compute_transform(A, B):

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A

    return R, t


def compute_error(A, B, R, t):

    errors = []

    for i in range(len(A)):

        pred = R @ A[i] + t
        err = np.linalg.norm(pred - B[i])
        errors.append(err)

    return np.mean(errors)


def main():

    rclpy.init()

    node = ObjectXYZ()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    import threading
    threading.Thread(target=executor.spin, daemon=True).start()

    print("Connecting robot...")

    robot = Robot(ROBOT_IP)

    camera_points = []
    robot_points = []

    print("\nCalibration started")

    while True:

        input("\nPlace robot TCP on phone center then press ENTER")

        cam_p = node.capture()

        if cam_p is None:
            print("Camera detection failed")
            continue

        rob_p = get_robot_xyz(robot)

        print("Camera:", cam_p)
        print("Robot :", rob_p)

        camera_points.append(cam_p)
        robot_points.append(rob_p)

        print("Pairs collected:", len(camera_points))

        cmd = input("Add more points? (y/n): ")

        if cmd.lower() != "y":
            break


    A = np.array(camera_points)
    B = np.array(robot_points)

    print("\nSolving transform...")

    R, t = compute_transform(A,B)

    err = compute_error(A,B,R,t)

    print("\nRotation Matrix:")

    print(R)

    print("\nTranslation Vector:")

    print(t)

    print("\nMean calibration error (meters):")

    print(err)

    print("\nTransform camera → robot")

    print("robot_point = R * camera_point + t")


if __name__ == "__main__":
    main()
