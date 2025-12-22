#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Subscribers
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10)
        self.subscription_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10)
            
        # Publisher
        self.publisher_pose = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.publisher_mask = self.create_publisher(Image, '/perception/mask', 10)
        
        # Tools
        self.br = CvBridge()
        
        # State
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.camera_frame_id = "panda_hand_camera" # Default, updated from msg
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info('Perception Node Started. Waiting for images...')

    def info_callback(self, msg):
        # We only need to get intrinsics once
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(f'Camera Intrinsics Matrix:\n{self.camera_intrinsics}')
            self.get_logger().info(f'Camera Info Frame ID: {self.camera_frame_id}')
            # Optional: Unsubscribe? Or keep it in case it changes (unlikely)
            self.destroy_subscription(self.subscription_info)

    def depth_callback(self, msg):
        try:
            # Isaac Sim publishes depth as 32FC1 usually, but let's be safe
            self.latest_depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth Callback Error: {e}')

    def image_callback(self, msg):
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            # Log warning every 5 seconds if waiting for dependencies
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds > 5e9:
                self.get_logger().warn('Waiting for Depth or Camera Info...')
                self.last_log_time = now
            return

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            
            # Color Thresholding for Red Cube
            # HSV is often better for color
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Red color definition in HSV (Red wraps around 0/180)
            # Generic Red masks
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            # Publish Debug Mask
            debug_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
            debug_msg.header = msg.header
            self.publisher_mask.publish(debug_msg)
            
            # Find Contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour (assume it is the cube)
                c = max(contours, key=cv2.contourArea)
                
                if cv2.contourArea(c) > 500: # Minimum area filter
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Get Depth at centroid
                        # Ensure bounds
                        depth_h, depth_w = self.latest_depth_image.shape
                        if 0 <= cy < depth_h and 0 <= cx < depth_w:
                            d = self.latest_depth_image[cy, cx]
                            
                            # Isaac Sim Depth might be Inf or 0 if invalid
                            if not np.isinf(d) and not np.isnan(d) and d > 0:
                                # Back-project to 3D Camera coordinates
                                # [u, v, 1] * Z = K * [X, Y, Z]
                                # X = (u - cx_k) * Z / fx
                                # Y = (v - cy_k) * Z / fy
                                # Z = d
                                
                                fx = self.camera_intrinsics[0, 0]
                                fy = self.camera_intrinsics[1, 1]
                                cx_k = self.camera_intrinsics[0, 2]
                                cy_k = self.camera_intrinsics[1, 2]
                                
                                Z = d
                                X = (cx - cx_k) * Z / fx
                                Y = (cy - cy_k) * Z / fy
                                
                                # Publish Pose
                                pose_msg = PoseStamped()
                                pose_msg.header.stamp = self.get_clock().now().to_msg()
                                pose_msg.header.frame_id = self.camera_frame_id
                                
                                pose_msg.pose.position.x = float(X)
                                pose_msg.pose.position.y = float(Y)
                                pose_msg.pose.position.z = float(Z)
                                
                                # Default orientation (identity)
                                pose_msg.pose.orientation.w = 1.0
                                
                                self.publisher_pose.publish(pose_msg)
                                # self.get_logger().info(f'Object detected at (Camera Frame): X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}')
            else:
                 # Log failure to find contours periodically
                 now = self.get_clock().now()
                 if (now - self.last_log_time).nanoseconds > 2e9: # 2 seconds
                     self.get_logger().info('No red object detected in image.')
                     self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'Image Callback Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
