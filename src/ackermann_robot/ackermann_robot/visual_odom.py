#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R_convert
import math

class VisualOdom(Node):
    def __init__(self):
        super().__init__('visual_odom_node')
        
        # --- Parameters (Tuned for Repetitive Environments) ---
        self.min_features = 2000 # Increased for more "chances" to match
        self.match_threshold = 10 # Low threshold to keep tracking alive
        self.init_threshold = 50  # High threshold to start tracking (hysteresis)
        self.downscale = 0.5 # Downscale factor for performance
        
        # --- State ---
        self.prev_image = None
        self.prev_kp = None
        self.prev_des = None
        self.K = None
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        
        self.current_speed = 0.0
        self.last_img_time = None

        # --- Tools ---
        self.bridge = CvBridge()
        # ORB with more levels handles scale changes (ramps) better
        self.orb = cv2.ORB_create(nfeatures=self.min_features, scaleFactor=1.2, nlevels=8)
        
        # Brute Force Matcher with Hamming distance (specific for ORB)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # --- Sub/Pub ---
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.info_cb, 30)
        self.create_subscription(Image, '/front_camera/image', self.image_cb, 30)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 30)
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 30)

        self.get_logger().info("Visual Odom Node Active: Zero-Match Handling Enabled.")

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera Info Received. Calibration set.")

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx**2 + vy**2)

    def image_cb(self, msg):
        if self.K is None:
            self.get_logger().warn("Waiting for Camera Info...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            if self.downscale < 1.0:
                width = int(cv_image.shape[1] * self.downscale)
                height = int(cv_image.shape[0] * self.downscale)
                cv_image = cv2.resize(cv_image, (width, height))
            curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().error(f"Image Conversion Error: {e}")
            return

        # 1. Feature Detection
        kp, des = self.orb.detectAndCompute(cv_image, None)
        
        # Guard against frames with no features (e.g. looking at a plain white wall)
        if des is None:
            return

        # --- Initialization Logic ---
        if self.prev_image is None:
            if len(kp) < self.init_threshold:
                self.get_logger().warn(f"Waiting for better frame to init ({len(kp)}/{self.init_threshold} features)...", throttle_duration_sec=1.0)
                return
            
            self.get_logger().info("Tracker initialized. Setting reference frame.")
            self.prev_image = cv_image
            self.prev_kp = kp
            self.prev_des = des
            self.last_img_time = curr_time
            return

        # --- Tracking Logic ---
        # 2. Feature Matching
        matches = self.bf.match(self.prev_des, des)
        matches = sorted(matches, key=lambda x: x.distance)

        # --- Zero Match Logic ---
        if len(matches) < self.match_threshold:
            self.get_logger().warn(f"Insufficient matches ({len(matches)}). Tracking lost, resetting tracker.")
            self.prev_image = None
            return

        # 3. Motion Estimation
        pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

        # Adjust K for downscaling
        K_curr = self.K.copy()
        if self.downscale < 1.0:
            K_curr[:2, :] *= self.downscale

        # Increased threshold to 2.0 to be more forgiving
        E, mask = cv2.findEssentialMat(pts2, pts1, K_curr, method=cv2.RANSAC, prob=0.999, threshold=2.0)
            
        if E is not None:
            _, R_rel, t_rel, mask = cv2.recoverPose(E, pts2, pts1, K_curr)

            if self.last_img_time is not None:
                dt = curr_time - self.last_img_time
                scale = self.current_speed * dt
                # --- THE FIX: Camera Frame to ROS Frame Mapping ---
                # Camera (OpenCV): X=Right, Y=Down, Z=Forward
                # Robot (ROS): X=Forward, Y=Left, Z=Up

                # 1. Remap Translation (t_rel) from Camera to ROS
                t_ros = np.zeros((3, 1))
                t_ros[0] =  t_rel[2]  # Depth -> Forward
                t_ros[1] = -t_rel[0]  # Width -> Left
                t_ros[2] = -t_rel[1]  # Height -> Up

                # 2. Remap Rotation Matrix (R_rel)
                # We need to change the basis of the rotation matrix to match ROS
                # This prevents the 90-degree "pitch up" effect
                R_ros = np.eye(3)
                R_ros[0, 0] =  R_rel[2, 2]
                R_ros[0, 1] = -R_rel[2, 0]
                R_ros[0, 2] = -R_rel[2, 1]

                R_ros[1, 0] = -R_rel[0, 2]
                R_ros[1, 1] =  R_rel[0, 0]
                R_ros[1, 2] =  R_rel[0, 1]

                R_ros[2, 0] = -R_rel[1, 2]
                R_ros[2, 1] =  R_rel[1, 0]
                R_ros[2, 2] =  R_rel[1, 1]

                # 3. Update Global Pose
                if scale > 0.001: 
                    # Move the robot in its current world orientation
                    self.cur_t = self.cur_t + self.cur_R.dot(t_ros) * scale
                    # Update the world orientation
                    self.cur_R = self.cur_R.dot(R_ros)
            self.publish_odom(msg.header.stamp)

        # 4. Successful Track Update (Update reference for next frame)
        self.prev_image = cv_image
        self.prev_kp = kp
        self.prev_des = des
        self.last_img_time = curr_time

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(self.cur_t[0])
        msg.pose.pose.position.y = float(self.cur_t[1])
        msg.pose.pose.position.z = float(self.cur_t[2])

        q = R_convert.from_matrix(self.cur_R).as_quat()
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # High covariance when using mono-VO (0.1 is standard)
        msg.pose.covariance[0] = 0.1 
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[14] = 0.1
        self.odom_pub.publish(msg)
        self.get_logger().info(f"Odom: x={float(self.cur_t[0]):.2f}, y={float(self.cur_t[1]):.2f}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()