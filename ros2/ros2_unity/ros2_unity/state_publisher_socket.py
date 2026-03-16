#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import socket
import struct
import time

import ros2_unity.robot_connection as rc
from fairino import Robot


ROBOT_IP = '192.168.58.2'
STATE_PORT = 20003

# TODO: set these according to the documentation of port 20003
STATE_PACKET_SIZE = 512  # bytes, example placeholder
STATE_PACKET_FMT  = '<5i 6f 6f 6f 6f 2f 6f 3?'  # example only!


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.declare_parameter('publish_rate', 50.0)
        rate = self.get_parameter('publish_rate').value

        # Optional: keep SDK connection for motion / non‑realtime calls
        self.robot = Robot.RPC(ROBOT_IP)

        self.pub   = self.create_publisher(String, '/nonrt_state_data', 10)

        self._latest = None
        self._lock   = threading.Lock()
        self._stop   = threading.Event()

        # TCP socket to realtime state port
        self._sock = None
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'State publisher ready @ {rate}Hz')

    # ---- Socket helpers -------------------------------------------------

    def _connect_socket(self):
        while not self._stop.is_set():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(2.0)
                s.connect((ROBOT_IP, STATE_PORT))
                s.settimeout(None)
                self.get_logger().info('Connected to realtime state port')
                return s
            except Exception as e:
                self.get_logger().warn(f'State socket connect failed: {e}')
                time.sleep(1.0)
        return None

    def _recv_exact(self, n):
        """Receive exactly n bytes or None if connection lost."""
        buf = b''
        while len(buf) < n and not self._stop.is_set():
            try:
                chunk = self._sock.recv(n - len(buf))
                if not chunk:
                    return None
                buf += chunk
            except Exception:
                return None
        return buf if len(buf) == n else None

    # ---- Socket reader loop ---------------------------------------------

    def _read_loop(self):
        while not self._stop.is_set():
            if self._sock is None:
                self._sock = self._connect_socket()
                if self._sock is None:
                    break

            try:
                raw = self._recv_exact(STATE_PACKET_SIZE)
                if raw is None:
                    self.get_logger().warn('State socket disconnected, reconnecting...')
                    self._sock.close()
                    self._sock = None
                    time.sleep(0.5)
                    continue

                # Decode one state frame
                data = self._parse_state_packet(raw)

                with self._lock:
                    self._latest = data

            except Exception as e:
                self.get_logger().warn(f'State read error: {e}')
                if self._sock:
                    try:
                        self._sock.close()
                    except Exception:
                        pass
                    self._sock = None
                time.sleep(0.5)

    # ---- Packet parsing -------------------------------------------------

    def _parse_state_packet(self, raw: bytes):
        """Parse one binary state packet from port 20003.

        IMPORTANT: You must adapt STATE_PACKET_FMT and this mapping to
        the actual layout given in your Fairino manual.
        """
        # Example unpack, adjust to real structure!
        vals = struct.unpack(STATE_PACKET_FMT, raw)

        # Example mapping indices; set these to match your protocol
        program_state = int(vals[0])
        robot_state   = int(vals[1])
        robot_mode    = int(vals[2])
        main_code     = int(vals[3])
        sub_code      = int(vals[4])

        offset = 5
        jt_cur_pos = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6
        tl_cur_pos = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6
        flange_cur_pos = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6
        actual_qd = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6
        jt_cur_tor = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6
        actual_TCP_CmpSpeed = [round(float(v), 3) for v in vals[offset:offset+2]]
        offset += 2
        actual_TCP_Speed = [round(float(v), 3) for v in vals[offset:offset+6]]
        offset += 6

        emergency_stop  = bool(vals[offset]); offset += 1
        motion_done     = bool(vals[offset]); offset += 1
        collision_state = bool(vals[offset]); offset += 1

        # If your packet includes mc_queue_len, tool, user, etc., continue mapping:
        mc_queue_len = 0
        tool = 0
        user = 0

        data = {
            'program_state':  program_state,
            'robot_state':    robot_state,
            'robot_mode':     robot_mode,
            'main_code':      main_code,
            'sub_code':       sub_code,
            'jt_cur_pos':     jt_cur_pos,
            'tl_cur_pos':     tl_cur_pos,
            'flange_cur_pos': flange_cur_pos,
            'actual_qd':      actual_qd,
            'jt_cur_tor':     jt_cur_tor,
            'actual_TCP_CmpSpeed': actual_TCP_CmpSpeed,
            'actual_TCP_Speed':    actual_TCP_Speed,
            'emergency_stop':  emergency_stop,
            'motion_done':     motion_done,
            'collision_state': collision_state,
            'mc_queue_len':    mc_queue_len,
            'tool': tool,
            'user': user,
        }
        return data

    # ---- ROS2 publish / shutdown ---------------------------------------

    def _publish(self):
        with self._lock:
            data = self._latest
        if data is None:
            return
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self._sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rc.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

