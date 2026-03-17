#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D

class SimpleDetector(Node):

    def __init__(self):
        super().__init__("simple_detector")

        self.bridge = CvBridge()

        self.cameras = [
            "front_camera",
            "left_camera",
            "right_camera",
            "rear_camera"
        ]

        self.subs = []
        self.pubs = {}

        # simple foreground detector
        self.bg = cv2.createBackgroundSubtractorMOG2()

        for cam in self.cameras:

            self.subs.append(
                self.create_subscription(
                    Image,
                    f"/{cam}/image",
                    lambda msg, c=cam: self.image_cb(msg, c),
                    10
                )
            )

            self.pubs[cam] = self.create_publisher(
                Detection2DArray,
                f"/detections/{cam}",
                10
            )

        self.get_logger().info("Simple detector running")

    def image_cb(self, msg, cam):

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        fg = self.bg.apply(img)

        contours, _ = cv2.findContours(
            fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        out = Detection2DArray()
        out.header = msg.header

        for c in contours:

            if cv2.contourArea(c) < 500:
                continue

            x,y,w,h = cv2.boundingRect(c)

            det = Detection2D()

            bbox = BoundingBox2D()
            bbox.center.position.x = float(x + w/2)
            bbox.center.position.y = float(y + h/2)
            bbox.size_x = float(w)
            bbox.size_y = float(h)

            det.bbox = bbox

            out.detections.append(det)

        self.pubs[cam].publish(out)


def main():
    rclpy.init()
    node = SimpleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()