#!/usr/bin/env python3
"""Extract camera extrinsics from TF."""
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class CameraTransformExtractor(Node):
    """Extract camera extrinsics from TF."""

    def __init__(self):
        """Initialize the CameraTransformExtractor node."""
        super().__init__('camera_transform_extractor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Change these names to match your URDF exactly
        self.base_frame = 'base_link'
        # Based on visual_odom.py, only the front camera appears to be in use.
        # If your robot has other cameras, add their link names to this list.
        self.cameras = [
            'front_camera_link', 'rear_camera_link',
            'left_camera_link', 'right_camera_link',
        ]

        self.timer = self.create_timer(2.0, self.get_transforms)

    def get_transforms(self):
        """Lookup and log transforms for cameras."""
        for cam in self.cameras:
            try:
                # Lookup transform from Rear Axle (Base) to Camera
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    self.base_frame, cam, now)

                # Translation
                tx = trans.transform.translation.x
                ty = trans.transform.translation.y
                tz = trans.transform.translation.z

                # Rotation (Quaternion to Matrix)
                quat = trans.transform.rotation
                r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
                rot_matrix = r.as_matrix()

                # Construct 4x4 Row-Major Matrix
                # [R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz, 0, 0, 0, 1]
                matrix = [
                    rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2], tx,
                    rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2], ty,
                    rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2], tz,
                    0.0, 0.0, 0.0, 1.0
                ]

                self.get_logger().info(
                    f'\n{cam} transformation_matrix:\n{list(matrix)}')

            except Exception as e:
                self.get_logger().warn(f'Could not find transform for {cam}: {e}')


def main():
    """Run the CameraTransformExtractor node."""
    rclpy.init()
    node = CameraTransformExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()