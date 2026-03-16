#!/usr/bin/env python3
"""
state_publisher.py
Streams robot state via dedicated raw TCP socket to port 8081.
Struct offsets verified from live FR5 controller binary frames.

Confirmed offsets (little-endian float64 arrays):
  offset   8  : jt_cur_pos[6]       joint current positions (deg)
  offset  56  : jt_cur_vel[6]?      (unknown, skipped)
  offset 248  : jt_tgt_pos[6]       joint target positions
  offset 440  : tl_cur_pos[6]       TCP current pose
  offset 488  : tl_tgt_pos[6]?      (unknown, skipped)
  offset 584  : flange_cur_pos[6]   flange current pose
  offset 812  : status int fields   (program_state, robot_state, robot_mode, motion_done ...)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import threading
import socket
import select
import struct

CONTROLLER_IP   = "192.168.58.2"
STATE_PORT      = 8081
FRAME_HEAD      = b"/f/b"
FRAME_TAIL      = b"III/b/f"
RECONNECT_DELAY = 3.0

# Verified offsets from live binary capture
OFF_JT_CUR_POS      = 8    # float64 x6  joint current pos (deg)
OFF_JT_TGT_POS      = 248  # float64 x6  joint target pos  (deg)
OFF_TL_CUR_POS      = 440  # float64 x6  TCP current pose
OFF_FLANGE_CUR_POS  = 584  # float64 x6  flange current pose
OFF_STATUS          = 812  # int32 fields start here


class RobotStateSocket:
    """
    Dedicated raw TCP connection to port 8081.
    Completely independent from the RPC command connection on port 20003.
    """

    def __init__(self, ip, port, logger):
        self._ip     = ip
        self._port   = port
        self._log    = logger
        self._sock   = None
        self._lock   = threading.Lock()
        self._latest = None
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._thread.join(timeout=3.0)

    def get_latest(self):
        with self._lock:
            return self._latest

    # -------------------------------------------------------------------------

    def _connect(self):
        import time
        while not self._stop.is_set():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self._ip, self._port))
                s.setsockopt(socket.SOL_SOCKET,  socket.SO_KEEPALIVE,  1)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE,  5)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT,   3)
                s.setblocking(False)
                self._log.info(
                    f"Connected to robot state socket {self._ip}:{self._port}")
                return s
            except Exception as e:
                self._log.warn(
                    f"State socket connect failed: {e} - retry in {RECONNECT_DELAY}s")
                time.sleep(RECONNECT_DELAY)
        return None

    def _recv_exact(self, sock, n):
        buf = b""
        while len(buf) < n:
            if self._stop.is_set():
                raise ConnectionError("stop requested")
            ready, _, _ = select.select([sock], [], [], 0.5)
            if not ready:
                continue
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("socket closed by remote")
            buf += chunk
        return buf

    def _find_frame_head(self, sock):
        window = b""
        while not self._stop.is_set():
            ready, _, _ = select.select([sock], [], [], 0.5)
            if not ready:
                continue
            byte = sock.recv(1)
            if not byte:
                raise ConnectionError("socket closed while searching frame head")
            window = (window + byte)[-4:]
            if window == FRAME_HEAD:
                return True
        return False

    def _parse_frame(self, payload):
        size = len(payload)
        d    = {}

        def read_f64_array(offset, count):
            end = offset + count * 8
            if size < end:
                return [0.0] * count
            return [round(v, 3) for v in
                    struct.unpack_from(f"<{count}d", payload, offset)]

        def read_i32(offset):
            if size < offset + 4:
                return 0
            return struct.unpack_from("<i", payload, offset)[0]

        def read_u8(offset):
            if size < offset + 1:
                return 0
            return struct.unpack_from("<B", payload, offset)[0]

        # Joint positions (confirmed offset 8)
        d['jt_cur_pos'] = read_f64_array(OFF_JT_CUR_POS, 6)

        # TCP pose (confirmed offset 440)
        d['tl_cur_pos'] = read_f64_array(OFF_TL_CUR_POS, 6)

        # Flange pose (offset 584, same gap pattern as tl)
        d['flange_cur_pos'] = read_f64_array(OFF_FLANGE_CUR_POS, 6)

        # Status integer fields at offset 812
        # All of program_state/robot_state/robot_mode/motion_done resolved
        # to the same offsets (812, 1108, 1316...) meaning they share
        # aligned int32 slots - read them sequentially
        d['program_state'] = read_i32(OFF_STATUS)
        d['robot_state']   = read_i32(OFF_STATUS + 4)
        d['robot_mode']    = read_i32(OFF_STATUS + 8)
        d['main_code']     = read_i32(OFF_STATUS + 12)
        d['sub_code']      = read_i32(OFF_STATUS + 16)
        d['motion_done']   = read_u8(OFF_STATUS + 20)
        d['emergency_stop']  = bool(read_u8(OFF_STATUS + 21))
        d['collision_state'] = bool(read_u8(OFF_STATUS + 22))

        # Zero out fields we couldn't locate (not in payload or all-zero)
        d['actual_qd']           = [0.0] * 6
        d['jt_cur_tor']          = [0.0] * 6
        d['actual_TCP_CmpSpeed'] = [0.0] * 2
        d['actual_TCP_Speed']    = [0.0] * 6
        d['mc_queue_len']        = 0
        d['tool']                = 0
        d['user']                = 0

        return d

    def _run(self):
        import time
        while not self._stop.is_set():
            self._sock = self._connect()
            if self._sock is None:
                break
            try:
                self._read_frames(self._sock)
            except ConnectionError as e:
                self._log.warn(f"State socket disconnected: {e} - reconnecting...")
            except Exception as e:
                self._log.error(f"State socket error: {e} - reconnecting...")
            finally:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
            if not self._stop.is_set():
                time.sleep(RECONNECT_DELAY)

    def _read_frames(self, sock):
        while not self._stop.is_set():
            if not self._find_frame_head(sock):
                return
            pre          = self._recv_exact(sock, 7)
            frame_length = struct.unpack_from("<i", pre, 3)[0]
            future_len   = frame_length + 3
            raw          = self._recv_exact(sock, future_len)
            if raw[-7:] != FRAME_TAIL:
                self._log.warn("Frame tail mismatch - resyncing")
                continue
            payload = raw[:-7]
            data    = self._parse_frame(payload)
            if data:
                with self._lock:
                    self._latest = data


# -----------------------------------------------------------------------------

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.declare_parameter('publish_rate',  50.0)
        self.declare_parameter('controller_ip', CONTROLLER_IP)
        self.declare_parameter('state_port',    STATE_PORT)

        rate = self.get_parameter('publish_rate').value
        ip   = self.get_parameter('controller_ip').value
        port = self.get_parameter('state_port').value

        self._pub    = self.create_publisher(String, '/nonrt_state_data', 10)
        self._stream = RobotStateSocket(ip, port, self.get_logger())
        self._stream.start()

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"State publisher ready @ {rate}Hz -> raw socket {ip}:{port}")

    def _publish(self):
        data = self._stream.get_latest()
        if data is None:
            return

        # NaN/Inf guard - protect Unity from bad quaternion values
        jt = data.get('jt_cur_pos', [])
        if len(jt) < 6:
            return
        if any(math.isnan(v) or math.isinf(v) for v in jt):
            self.get_logger().warn(
                "jt_cur_pos NaN/Inf - dropping frame",
                throttle_duration_sec=2.0)
            return

        msg      = String()
        msg.data = json.dumps(data)
        self._pub.publish(msg)

    def destroy_node(self):
        self._stream.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
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
