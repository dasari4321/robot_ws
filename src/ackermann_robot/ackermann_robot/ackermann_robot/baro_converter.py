#!/usr/bin/env python3
"""Convert fluid pressure to altitude."""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure


class ScaledBaroNode(Node):
    """Convert fluid pressure to altitude."""

    def __init__(self):
        """Initialize the ScaledBaroNode."""
        super().__init__('baro_converter')

        # --- TUNING PARAMETER ---
        # If your floor is 4m but code says 0.2m, your scale is off by 20x.
        # Standard sea level is ~0.083. Try increasing this.
        # Adjusted for your current observation
        self.pressure_to_meters_scale = 0.0833 * 20.0

        self.p_ref = 101325.0
        self.z_offset = None

        self.sub_ref = self.create_subscription(
            FluidPressure, '/env/reference_pressure', self.ref_cb, 10)
        self.sub_robot = self.create_subscription(
            FluidPressure, '/air_pressure', self.robot_cb, 10)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/baro_pose', 10)

    def ref_cb(self, msg):
        """Callback for reference pressure."""
        self.p_ref = msg.fluid_pressure

    def robot_cb(self, msg):
        """Callback for robot pressure and publishing altitude."""
        # 1. Calculate pressure delta
        delta_p = self.p_ref - msg.fluid_pressure

        # 2. Convert to meters using our scale
        raw_z = delta_p * self.pressure_to_meters_scale

        # 3. Handle Initial Zeroing
        if self.z_offset is None:
            self.z_offset = raw_z
            return

        final_z = raw_z - self.z_offset

        # 4. Publish
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.pose.pose.position.z = final_z
        out.pose.covariance[14] = 0.01
        self.pub.publish(out)

        self.get_logger().info(
            f'Pressure Delta: {delta_p:.2f} Pa | Scaled Z: {final_z:.3f}m')


def main(args=None):
    """Run the ScaledBaroNode."""
    rclpy.init(args=args)
    node = ScaledBaroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
