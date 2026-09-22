#!/usr/bin/env python3
"""Use case 2: record /flair/image_raw to disk as a dataset.

Directly serves the TX project's need for benchmark datasets (images +
a manifest to correlate frames with an external reference, e.g. VRPN,
added later). Saves one PNG per (kept) frame plus a manifest.csv with
index, ROS timestamp (ns) and filename.

Stop recording with Ctrl+C: it prints a short recap (frames saved,
output directory) instead of a raw error/traceback.
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        self.bridge = CvBridge()

        self.declare_parameter('input_topic', '/flair/image_raw')
        self.declare_parameter('output_dir', os.path.expanduser('~/flair_datasets'))
        self.declare_parameter('save_every_n_frames', 1)  # 1 = save every frame
        self.declare_parameter('status_period_sec', 2.5)

        in_topic = self.get_parameter('input_topic').value
        base_dir = self.get_parameter('output_dir').value
        self.save_every_n = max(1, self.get_parameter('save_every_n_frames').value)
        status_period = self.get_parameter('status_period_sec').value

        run_name = time.strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(base_dir, run_name)
        self.images_dir = os.path.join(self.run_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=True)

        self.manifest_path = os.path.join(self.run_dir, 'manifest.csv')
        self.manifest_file = open(self.manifest_path, 'w', newline='')
        self.manifest_writer = csv.writer(self.manifest_file)
        self.manifest_writer.writerow(['index', 'stamp_ns', 'filename'])

        self.frame_index = 0
        self.saved_count = 0
        self.start_time = time.monotonic()
        self._closed = False

        self.subscription = self.create_subscription(
            Image, in_topic, self.on_image, 10)
        self.status_timer = self.create_timer(status_period, self.on_status_timer)

        print(f'Recording dataset to: {self.run_dir}')
        print('Press Ctrl+C to stop.\n')

    def on_image(self, msg: Image):
        if self.frame_index % self.save_every_n == 0:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            filename = f'frame_{self.saved_count:06d}.png'
            cv2.imwrite(os.path.join(self.images_dir, filename), frame)
            self.manifest_writer.writerow([self.saved_count, stamp_ns, filename])
            self.manifest_file.flush()
            self.saved_count += 1

        self.frame_index += 1

    def on_status_timer(self):
        elapsed = time.monotonic() - self.start_time
        print(f'  ... {self.saved_count} images captured ({elapsed:.0f}s elapsed)')

    def close(self):
        """Flush and close the manifest, print a short recap. Safe to call
        more than once (Ctrl+C can otherwise trigger this twice)."""
        if self._closed:
            return
        self._closed = True

        self.manifest_file.close()
        elapsed = time.monotonic() - self.start_time
        print('\nRecording stopped.')
        print(f'  {self.saved_count} images saved')
        print(f'  duration: {elapsed:.1f}s')
        print(f'  output:   {self.run_dir}')
        print(f'  manifest: {self.manifest_path}')


def main():
    rclpy.init()
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()