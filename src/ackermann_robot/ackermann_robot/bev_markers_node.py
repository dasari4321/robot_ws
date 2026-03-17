#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class MarkerNode(Node):

    def __init__(self):

        super().__init__("bev_markers")

        self.create_subscription(
            OccupancyGrid,
            "/bev/semantic_grid",
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            "/bev/markers",
            10
        )

    def cb(self, msg):

        markers = MarkerArray()

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution

        for i, val in enumerate(msg.data):

            if val < 100:
                continue

            x = (i % width) * res + msg.info.origin.position.x
            y = (i // width) * res + msg.info.origin.position.y

            m = Marker()

            m.header.frame_id = "base_link"
            m.id = i
            m.type = Marker.CUBE

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1

            m.scale.x = res
            m.scale.y = res
            m.scale.z = 0.2

            m.color.r = 1.0
            m.color.a = 1.0

            markers.markers.append(m)

        self.pub.publish(markers)


def main():

    rclpy.init()

    node = MarkerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()