#!/usr/bin/env python3
"""Use case 1: dense optical flow (Farneback) computed on /flair/image_raw,
republished as a motion-vector overlay on /flair/optical_flow_view.

This is a demo/validation node, not a benchmark: it shows that the streamed
images are usable as-is for a standard OpenCV optical flow algorithm, with
no extra preprocessing needed.

At 60 Hz, frame-to-frame displacement is often sub-pixel, which is too
close to the estimator's noise floor to visualize meaningfully. This node
compares frames `frame_stride` apart (more visible motion) and draws sparse
arrows on top of the original image (only where the magnitude clears
`min_magnitude`), instead of the dense HSV color-wheel encoding, which is
harder to read and gets swamped by noise when the real signal is small.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class OpticalFlowDemo(Node):
    def __init__(self):
        super().__init__('optical_flow_demo')
        self.bridge = CvBridge()
        self.prev_gray = None
        self.frame_count = 0

        self.declare_parameter('input_topic', '/flair/image_raw')
        self.declare_parameter('output_topic', '/flair/optical_flow_view')
        # compare frame N against frame N-frame_stride: at 60 Hz, 1-frame
        # displacement is often sub-pixel (too close to noise floor)
        self.declare_parameter('frame_stride', 3)
        # grid spacing (px) between drawn arrows
        self.declare_parameter('arrow_step', 16)
        # arrows below this magnitude (px, over frame_stride frames) are not drawn
        self.declare_parameter('min_magnitude', 1.0)
        # multiply flow vectors by this before drawing, for visibility
        self.declare_parameter('arrow_scale', 3.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_stride = max(1, self.get_parameter('frame_stride').value)
        self.arrow_step = self.get_parameter('arrow_step').value
        self.min_magnitude = self.get_parameter('min_magnitude').value
        self.arrow_scale = self.get_parameter('arrow_scale').value

        self.publisher = self.create_publisher(Image, out_topic, 10)
        self.subscription = self.create_subscription(
            Image, in_topic, self.on_image, 10)

        self.get_logger().info(
            f'subscribing to {in_topic}, publishing flow visualization on {out_topic} '
            f'(frame_stride={self.frame_stride})')

    def on_image(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.frame_stride != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray, gray, None,
                pyr_scale=0.5, levels=3, winsize=15,
                iterations=3, poly_n=5, poly_sigma=1.2, flags=0)

            vis = self.draw_flow_arrows(frame, flow)

            out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(out_msg)

        self.prev_gray = gray

    def draw_flow_arrows(self, img, flow):
        h, w = flow.shape[:2]
        step = self.arrow_step
        vis = img.copy()

        ys, xs = np.mgrid[step // 2:h:step, step // 2:w:step]
        for y, x in zip(ys.ravel(), xs.ravel()):
            dx, dy = flow[y, x]
            magnitude = float(np.hypot(dx, dy))
            if magnitude < self.min_magnitude:
                continue
            x1 = int(x + dx * self.arrow_scale)
            y1 = int(y + dy * self.arrow_scale)
            cv2.arrowedLine(vis, (x, y), (x1, y1), (0, 255, 0), 1, tipLength=0.4)
            cv2.circle(vis, (x, y), 1, (0, 0, 255), -1)

        return vis


def main():
    rclpy.init()
    node = OpticalFlowDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
