#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sklearn.cluster import DBSCAN


class FusionNode(Node):

    def __init__(self):

        super().__init__("bev_fusion")

        self.points = []

        self.sub = self.create_subscription(
            PoseArray,
            "/objects/world",
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            PoseArray,
            "/bev/tracks",
            10
        )

    def cb(self, msg):

        pts = []

        for p in msg.poses:
            pts.append([p.position.x, p.position.y])

        if len(pts) == 0:
            return

        pts = np.array(pts)

        clustering = DBSCAN(eps=0.5, min_samples=1).fit(pts)

        labels = clustering.labels_

        centers = []

        for label in np.unique(labels):

            cluster = pts[labels == label]

            center = np.mean(cluster, axis=0)

            centers.append(center)

        out = PoseArray()

        for c in centers:

            p = Pose()
            p.position.x = c[0]
            p.position.y = c[1]

            out.poses.append(p)

        self.pub.publish(out)


def main():

    rclpy.init()

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()