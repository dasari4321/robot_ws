#!/usr/bin/env python3

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Imu

import message_filters

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class BevNode(Node):

    def __init__(self):

        super().__init__("bev_node")

        # ---------- PARAMETERS ----------

        self.declare_parameter("bev_res", 0.01)
        self.declare_parameter("bev_width", 300)
        self.declare_parameter("bev_height", 300)
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter(
            "camera_names",
            ["front_camera", "rear_camera", "left_camera", "right_camera"]
        )

        self.declare_parameter(
            "camera_links",
            ["front_camera_link", "rear_camera_link",
             "left_camera_link", "right_camera_link"]
        )

        self.res = self.get_parameter("bev_res").value
        self.w = self.get_parameter("bev_width").value
        self.h = self.get_parameter("bev_height").value
        self.base_frame = self.get_parameter("base_frame").value

        self.camera_names = self.get_parameter("camera_names").value
        self.camera_links = self.get_parameter("camera_links").value

        # ---------- TOOLS ----------

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- STATE ----------

        self.camera_info = {}
        self.maps = {}

        # ---------- PRECOMPUTE BEV GRID ----------

        u = np.arange(self.w)
        v = np.arange(self.h)

        self.uu, self.vv = np.meshgrid(u, v)
        self.roll = 0.0
        self.pitch = 0.0

        self.create_subscription(
            Imu,
            "/imu",
            self.imu_cb,
            10
        )
        # robot frame
        # +X forward
        # +Y left

        self.x_r = (self.h/2 - self.vv) * self.res
        self.y_r = (self.w/2 - self.uu) * self.res

        self.z_r = np.zeros_like(self.x_r)

        self.ones = np.ones_like(self.x_r)

        # self.points_base = np.stack([
        #     self.x_r.flatten(),
        #     self.y_r.flatten(),
        #     self.z_r.flatten(),
        #     self.ones.flatten()
        # ])

        # tilt correction from IMU
        tilt = R.from_euler("xy", [self.roll, self.pitch]).as_matrix()

        xyz = np.stack([
            self.x_r.flatten(),
            self.y_r.flatten(),
            self.z_r.flatten()
        ])

        xyz = tilt @ xyz

        self.points_base = np.vstack([xyz, self.ones.flatten()])

        # ---------- CAMERA INFO SUB ----------

        for i, cam in enumerate(self.camera_names):

            self.create_subscription(
                CameraInfo,
                f"/{cam}/camera_info",
                lambda msg, idx=i: self.info_cb(msg, idx),
                10
            )

        # ---------- IMAGE SYNC ----------

        subs = []

        for cam in self.camera_names:

            subs.append(
                message_filters.Subscriber(
                    self,
                    Image,
                    f"/{cam}/image",
                    qos_profile=qos_profile_sensor_data
                )
            )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            subs, 5, 0.1
        )

        self.sync.registerCallback(self.img_cb)

        # ---------- PUB ----------

        self.pub = self.create_publisher(Image, "/bev/image", 10)

        self.get_logger().info("BEV node ready")

    # --------------------------------------------------

    def info_cb(self, msg, idx):

        cam = self.camera_names[idx]

        if cam not in self.camera_info:

            self.camera_info[cam] = msg

            self.get_logger().info(f"{cam} intrinsics received")

    # --------------------------------------------------

    def get_transform(self, target, source):

        try:

            trans = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time()
            )

            t = trans.transform.translation
            r = trans.transform.rotation

            T = np.eye(4)

            T[:3,3] = [t.x,t.y,t.z]

            T[:3,:3] = R.from_quat(
                [r.x,r.y,r.z,r.w]
            ).as_matrix()

            return T

        except (LookupException, ConnectivityException, ExtrapolationException):

            return None

    # --------------------------------------------------

    def generate_lut(self, cam_idx):

        cam = self.camera_names[cam_idx]
        link = self.camera_links[cam_idx]

        info = self.camera_info[cam]

        # base -> camera

        T = self.get_transform(link, self.base_frame)

        if T is None:
            return False

        # convert link frame -> optical frame

        R_link_opt = np.array([
            [0,-1,0,0],
            [0,0,-1,0],
            [1,0,0,0],
            [0,0,0,1]
        ])

        T = R_link_opt @ T

        pts_cam = T @ self.points_base

        xyz = pts_cam[:3]

        valid = xyz[2] > 0.01

        xyz[:,~valid] = 0.1

        pts = xyz.T.reshape(-1,1,3)

        K = np.array(info.k).reshape(3,3)
        D = np.array(info.d)

        is_fisheye = info.distortion_model == "plumb_bob" or info.distortion_model == "equidistant" or info.distortion_model == "custom"


        K = np.array(info.k, dtype=np.float64).reshape(3,3)

        if is_fisheye:
            D = np.array(info.d[:4], dtype=np.float64)
            img_pts,_ = cv2.fisheye.projectPoints(
                pts,
                np.zeros(3),
                np.zeros(3),
                K,
                D
            )

        else:
            D = np.array(info.d, dtype=np.float64)
            img_pts,_ = cv2.projectPoints(
                pts,
                np.zeros(3),
                np.zeros(3),
                K,
                D
            )

        img_pts = img_pts.reshape(-1,2)

        map_x = img_pts[:,0].reshape(self.h,self.w).astype(np.float32)
        map_y = img_pts[:,1].reshape(self.h,self.w).astype(np.float32)

        mask = (
            (map_x >= 0) &
            (map_x < info.width) &
            (map_y >= 0) &
            (map_y < info.height) &
            valid.reshape(self.h,self.w)
        )

        weight = cv2.GaussianBlur(
            mask.astype(np.float32),
            (51,51),
            0
        )

        self.maps[cam] = (map_x,map_y,mask,weight)

        self.get_logger().info(f"LUT generated {cam}")

        return True

    # --------------------------------------------------

    def img_cb(self, *msgs):

        bev_sum = np.zeros((self.h,self.w,3),np.float32)
        weight_sum = np.zeros((self.h,self.w),np.float32)

        for i,msg in enumerate(msgs):

            cam = self.camera_names[i]

            if cam not in self.maps:

                if cam not in self.camera_info:
                    continue

                if not self.generate_lut(i):
                    continue

            mapx,mapy,mask,weight = self.maps[cam]

            img = self.bridge.imgmsg_to_cv2(msg,"bgr8")

            warped = cv2.remap(
                img,
                mapx,
                mapy,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )

            bev_sum += warped * weight[:,:,None]

            weight_sum += weight

        valid = weight_sum > 0

        bev = np.zeros_like(bev_sum,dtype=np.uint8)

        bev[valid] = (
            bev_sum[valid] /
            weight_sum[valid,None]
        ).astype(np.uint8)

        out = self.bridge.cv2_to_imgmsg(bev,"bgr8")

        out.header.frame_id = self.base_frame
        out.header.stamp = msgs[0].header.stamp

        self.pub.publish(out)

    def imu_cb(self, msg):

        q = msg.orientation

        rot = R.from_quat([q.x, q.y, q.z, q.w])

        roll, pitch, yaw = rot.as_euler("xyz")

        self.roll = roll
        self.pitch = pitch

def main():

    rclpy.init()

    node = BevNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

