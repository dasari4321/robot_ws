#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class MultiCameraSemanticBEV(Node):

    def __init__(self):

        super().__init__("semantic_bev_fusion")

        # ---------------- GRID SETTINGS ----------------

        self.resolution = 0.1
        self.size = 40.0

        self.grid_w = int(self.size / self.resolution)
        self.grid_h = int(self.size / self.resolution)

        self.base_frame = "base_link"

        self.grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)

        # ---------------- TF ----------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- CAMERA LIST ----------------

        self.cameras = [
            "front_camera",
            "left_camera",
            "right_camera",
            "rear_camera"
        ]

        # intrinsics example (replace with real)
        self.K = np.array([
            [300,0,320],
            [0,300,240],
            [0,0,1]
        ])

        self.camera_height = 0.12

        # ---------------- SUBSCRIBERS ----------------

        for cam in self.cameras:

            self.create_subscription(
                Detection2DArray,
                f"/detections/{cam}",
                lambda msg, c=cam: self.detection_cb(msg, c),
                10
            )

        # ---------------- PUBLISHER ----------------

        self.pub_grid = self.create_publisher(
            OccupancyGrid,
            "/bev/semantic_grid",
            10
        )

        self.get_logger().info("Multi-camera semantic BEV ready")

    # --------------------------------------------------
    # Pixel → Camera Ray
    # --------------------------------------------------

    def pixel_to_ray(self, u, v):

        ray = np.linalg.inv(self.K) @ np.array([u,v,1.0])

        return ray

    # --------------------------------------------------
    # Camera Ray → Ground Intersection
    # --------------------------------------------------

    def ray_ground_intersection(self, ray):

        t = -self.camera_height / ray[2]

        p = ray * t

        return p[0], p[1]

    # --------------------------------------------------
    # Detection callback
    # --------------------------------------------------

    def detection_cb(self, msg, camera_name):

        for det in msg.detections:

            bbox = det.bbox

            u = bbox.center.position.x
            v = bbox.center.position.y

            ray = self.pixel_to_ray(u, v)

            x_cam, y_cam = self.ray_ground_intersection(ray)

            x_world, y_world = self.transform_to_base(
                x_cam,
                y_cam,
                camera_name
            )

            self.update_grid(x_world, y_world)

        self.publish_grid()

    # --------------------------------------------------
    # Transform camera → base
    # --------------------------------------------------

    def transform_to_base(self, x, y, cam):

        try:

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                f"{cam}_link",
                rclpy.time.Time()
            )

            t = tf.transform.translation
            r = tf.transform.rotation

            Rm = R.from_quat([r.x,r.y,r.z,r.w]).as_matrix()

            p = np.array([x,y,0])

            p_base = Rm @ p + np.array([t.x,t.y,t.z])

            return p_base[0], p_base[1]

        except:

            return 0,0

    # --------------------------------------------------
    # Update BEV grid
    # --------------------------------------------------

    def update_grid(self, x, y):

        ix = int((x + self.size/2)/self.resolution)
        iy = int((y + self.size/2)/self.resolution)

        if 0 <= ix < self.grid_w and 0 <= iy < self.grid_h:

            self.grid[iy,ix] = 100

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

    node = MultiCameraSemanticBEV()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()