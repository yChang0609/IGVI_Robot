#!/usr/bin/env python3
"""
Relay node for Azure Kinect color images and camera_info.

Two fixes applied before passing data to rgbd_odometry / rtabmap:

1. bgra8 → bgr8: rtabmap's internal OpenCV code expects 3-channel BGR.
   Azure Kinect driver publishes 4-channel BGRA; the alpha channel causes
   rtabmap to create wrong-sized cv::Mat and crash with SIGSEGV.

2. rational_polynomial (8 coeff) → plumb_bob (5 coeff): rtabmap's
   CameraModel only handles the standard plumb_bob model.  The Kinect RGB
   camera uses the rational polynomial model with 8 coefficients (k1-k6,
   p1-p2).  Passing 8 coefficients into a plumb_bob code path causes an
   OpenCV assertion failure / SIGSEGV.  We truncate to 5 (k1,k2,p1,p2,k3)
   which is a close approximation for a well-calibrated sensor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge


class KinectRelay(Node):
    def __init__(self):
        super().__init__('kinect_relay')
        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, '/rgb/image_bgr8', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/rgb/camera_info_relay', 10)

        self.create_subscription(Image,      '/rgb/image_raw',   self._image_cb, 10)
        self.create_subscription(CameraInfo, '/rgb/camera_info', self._info_cb,  10)

    def _image_cb(self, msg: Image):
        if msg.encoding == 'bgra8':
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            bgr    = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
            out    = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            out.header = msg.header
        else:
            out = msg
        self.image_pub.publish(out)

    def _info_cb(self, msg: CameraInfo):
        if msg.distortion_model == 'rational_polynomial' and len(msg.d) > 5:
            msg.distortion_model = 'plumb_bob'
            msg.d = list(msg.d[:5])
        self.info_pub.publish(msg)


def main():
    rclpy.init()
    node = KinectRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
