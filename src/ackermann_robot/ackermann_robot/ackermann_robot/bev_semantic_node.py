#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


class SemanticBEVNode(Node):

    def __init__(self):

        super().__init__("semantic_bev")

        # ---------------- GRID SETTINGS ----------------

        self.resolution = 0.1
        self.size = 40.0

        self.grid_w = int(self.size / self.resolution)
        self.grid_h = int(self.size / self.resolution)

        self.base_frame = "base_link"

        self.grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)

        # ---------------- CAMERA INTRINSICS ----------------
        # replace with real values later

        self.K = np.array([
            [300, 0, 320],
            [0, 300, 240],
            [0, 0, 1]
        ])

        self.camera_height = 0.12

        # ---------------- SUBSCRIBERS ----------------

        self.create_subscription(
            Detection2DArray,
            "/detections",
            self.detection_cb,
            10
        )

        # ---------------- PUBLISHER ----------------

        self.pub_grid = self.create_publisher(
            OccupancyGrid,
            "/bev/semantic_grid",
            10
        )

        self.get_logger().info("Semantic BEV node running")

    # --------------------------------------------------
    # Pixel → Ground projection
    # --------------------------------------------------

    def project_to_ground(self, u, v):

        ray = np.linalg.inv(self.K) @ np.array([u, v, 1.0])

        t = -self.camera_height / ray[2]

        point = ray * t

        return point[0], point[1]

    # --------------------------------------------------
    # Detection callback
    # --------------------------------------------------

    def detection_cb(self, msg):

        self.grid[:] = 0

        for det in msg.detections:

            bbox = det.bbox

            u = bbox.center.position.x
            v = bbox.center.position.y

            x, y = self.project_to_ground(u, v)

            self.update_grid(x, y)

        self.publish_grid()

    # --------------------------------------------------

    def update_grid(self, x, y):

        ix = int((x + self.size/2) / self.resolution)
        iy = int((y + self.size/2) / self.resolution)

        if 0 <= ix < self.grid_w and 0 <= iy < self.grid_h:

            self.grid[iy, ix] = 100

    # --------------------------------------------------

    def publish_grid(self):

        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.frame_id = self.base_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h

        msg.info.origin.position.x = -self.size/2
        msg.info.origin.position.y = -self.size/2

        msg.data = self.grid.flatten().tolist()

        self.pub_grid.publish(msg)


def main():

    rclpy.init()

    node = SemanticBEVNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()