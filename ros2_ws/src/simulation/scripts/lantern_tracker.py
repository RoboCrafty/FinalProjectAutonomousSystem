#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3 
from cv_bridge import CvBridge
import cv2
import numpy as np

class LanternTracker(Node):
    def __init__(self):
        super().__init__('lantern_tracker')
        # Subscriptions
        self.semantic_sub = self.create_subscription(
            Image, '/Quadrotor/Sensors/SemanticCamera/image_raw', self.semantic_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/realsense/depth/image', self.depth_callback, 10)
        
        # Publisher
        self.target_pub = self.create_publisher(Vector3, '/lantern/target_data', 10)
        
        self.bridge = CvBridge()
        self.lantern_found = False
        self.lantern_error = 0.0
        self.lantern_area = 0.0

        self.get_logger().info("Lantern Tracker: 3D Avoidance Mode Active.")

    def semantic_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # Yellow Mask
            mask = cv2.inRange(hsv, np.array([25, 100, 100]), np.array([35, 255, 255]))
            moments = cv2.moments(mask)
            
            if moments["m00"] > 50:
                self.lantern_found = True
                self.lantern_error = float((moments["m10"] / moments["m00"]) - 160.0)
                self.lantern_area = float(moments["m00"])
            else:
                self.lantern_found = False
        except Exception as e:
            self.get_logger().error(f"Semantic Error: {e}")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            target_msg = Vector3()

            if self.lantern_found:
                target_msg.x = self.lantern_error
                target_msg.y = self.lantern_area
                target_msg.z = 1.0 # Mode: Hunt
            else:
                h, w = depth_norm.shape
                # Horizontal Avoidance (Left vs Right)
                left_side = np.mean(depth_norm[:, :w//2])
                right_side = np.mean(depth_norm[:, w//2:])
                target_msg.x = float(right_side - left_side)

                # Vertical Avoidance (Ceiling vs Floor)
                top_half = np.mean(depth_norm[:h//2, :])
                bottom_half = np.mean(depth_norm[h//2:, :])
                target_msg.y = float(top_half - bottom_half) # positive = climb
                
                target_msg.z = 2.0 # Mode: Search
            
            self.target_pub.publish(target_msg)
        except Exception as e:
            self.get_logger().error(f"Depth Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LanternTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
