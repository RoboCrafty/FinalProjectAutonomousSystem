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
        # Subscribing to the simulation's camera and depth feeds
        self.semantic_sub = self.create_subscription(
            Image, '/Quadrotor/Sensors/SemanticCamera/image_raw', self.semantic_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/realsense/depth/image', self.depth_callback, 10)
        
        # Publishing data to the hunting node and state machine
        self.target_pub = self.create_publisher(Vector3, '/lantern/target_data', 10)
        
        self.bridge = CvBridge()
        self.lantern_found = False
        self.lantern_x_err = 0.0
        self.lantern_y_err = 0.0 
        self.lantern_area = 0.0

    def semantic_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Yellow mask logic to isolate the lantern
            mask = cv2.inRange(hsv, np.array([25, 100, 100]), np.array([35, 255, 255]))
            moments = cv2.moments(mask)
            
            # Threshold for spotting a lantern (Area > 50)
            if moments["m00"] > 50: 
                self.lantern_found = True
                # Calculate pixel error from the center of the 320x240 image
                self.lantern_x_err = float((moments["m10"] / moments["m00"]) - 160.0)
                self.lantern_y_err = float((moments["m01"] / moments["m00"]) - 120.0)
                self.lantern_area = float(moments["m00"])
                
                # Real-time logging for debugging thresholds
                self.get_logger().info(f"Yellow Region Area: {self.lantern_area:.2f}")
            else:
                self.lantern_found = False
        except Exception as e:
            self.get_logger().error(f"Semantic error: {e}")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Normalize depth for brightness/gap analysis
            depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            h, w = depth_norm.shape

            # PRIORITY LOGIC: If a lantern is in sight, publish Hunt Data
            if self.lantern_found:
                hunt_msg = Vector3()
                hunt_msg.x = self.lantern_x_err # Horizontal Centering
                hunt_msg.y = self.lantern_area  # Area for speed/stop thresholds
                hunt_msg.z = self.lantern_y_err  # Vertical Tracking Error
                self.target_pub.publish(hunt_msg)
            
            # If no lantern is found, publish Exploration/Safety Data
            else:
                safety_msg = Vector3()
                # Corrected Syntax: Analyzing horizontal and vertical gaps in depth
                safety_msg.x = float(np.mean(depth_norm[:, w//2:]) - np.mean(depth_norm[:, :w//2])) 
                safety_msg.y = float(np.mean(depth_norm[:h//2, :]) - np.mean(depth_norm[h//2:, :]))
                # Signal "Mode 2.0" to the pilot
                safety_msg.z = 2.0 
                self.target_pub.publish(safety_msg)
                
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

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
