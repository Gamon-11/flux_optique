#!/usr/bin/env python3
"""Minimal ROS2 node that receives raw BGR images streamed by FLAIR over TCP
and republishes them as sensor_msgs/Image.

Wire protocol (must match ImageStreamer.cpp on the FLAIR side):
  - once, right after the TCP connection is established:
      8 bytes = width (uint32, little-endian) + height (uint32, little-endian)
  - then, for every frame:
      width * height * 3 raw bytes, BGR, row-major, no padding
"""
import socket
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Read exactly n bytes from a TCP socket (recv() can return fewer)."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("FLAIR closed the connection")
        buf.extend(chunk)
    return bytes(buf)


class FlairImageBridge(Node):
    def __init__(self):
        super().__init__('flair_image_bridge')

        self.declare_parameter('tcp_port', 5005)
        self.declare_parameter('topic', 'flair/image_raw')

        port = self.get_parameter('tcp_port').value
        topic = self.get_parameter('topic').value

        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(('0.0.0.0', port))
        self._server.listen(1)
        self.get_logger().info(
            f'listening for FLAIR on TCP port {port}, publishing on "{topic}"')

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while rclpy.ok():
            self.get_logger().info('waiting for FLAIR to connect...')
            conn, addr = self._server.accept()
            self.get_logger().info(f'FLAIR connected from {addr}')
            try:
                header = recv_exact(conn, 8)
                width, height = struct.unpack('<II', header)
                frame_size = width * height * 3
                self.get_logger().info(
                    f'image size announced by FLAIR: {width}x{height} (BGR)')

                while rclpy.ok():
                    raw = recv_exact(conn, frame_size)
                    frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher.publish(msg)
            except (ConnectionError, OSError) as e:
                self.get_logger().warn(f'FLAIR disconnected ({e}), waiting for a new connection')
            finally:
                conn.close()


def main():
    rclpy.init()
    node = FlairImageBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
