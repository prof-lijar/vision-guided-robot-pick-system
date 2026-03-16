#!/usr/bin/env python3
"""
robot_connection.py  —  Singleton RPC connection

Usage in any node:
    from ros2_unity.robot_connection import get_robot

    robot = get_robot()   # returns the shared Robot instance
    robot.StartJOG(...)
"""

from fairino import Robot
import threading

_robot = None
_lock  = threading.Lock()
_ip    = '192.168.58.2'


def init(ip: str = '192.168.58.2'):
    """Call once at startup (in main entry point) before any node uses get_robot()."""
    global _robot, _ip
    with _lock:
        if _robot is None:
            _ip    = ip
            _robot = Robot.RPC(ip)
            print(f'[RobotConnection] Connected to {ip}')
        else:
            print(f'[RobotConnection] Already connected to {_ip}')
    return _robot


def get_robot() -> Robot:
    """Get the shared robot instance. Raises if not yet initialised."""
    if _robot is None:
        raise RuntimeError(
            'Robot not initialised. Call robot_connection.init(ip) first.')
    return _robot


def close():
    global _robot
    with _lock:
        if _robot is not None:
            _robot.CloseRPC()
            _robot = None
            print('[RobotConnection] Disconnected')
