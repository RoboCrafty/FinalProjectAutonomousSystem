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
        self.semantic_sub = self.create_subscription(Image, '/Quadrotor/Sensors/SemanticCamera/image_raw', self.semantic_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/realsense/depth/image', self.depth_callback, 10)
        self.target_pub = self.create_publisher(Vector3, '/lantern/target_data', 10)
        self.bridge = CvBridge()
        self.lantern_found = False
        self.lantern_x_err = 0.0
        self.lantern_y_err = 0.0 
        self.lantern_area = 0.0

    def semantic_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([25, 100, 100]), np.array([35, 255, 255]))
            moments = cv2.moments(mask)
            if moments["m00"] > 50: 
                self.lantern_found = True
                self.lantern_x_err = float((moments["m10"] / moments["m00"]) - 160.0)
                self.lantern_y_err = float((moments["m01"] / moments["m00"]) - 120.0) # Vertical error
                self.lantern_area = float(moments["m00"])
            else:
                self.lantern_found = False
        except: pass

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            h, w = depth_norm.shape
            
            safety_msg = Vector3()
            safety_msg.x = float(np.mean(depth_norm[:, w//2:]) - np.mean(depth_norm[:, :w//2])) 
            safety_msg.y = float(np.mean(depth_norm[:h//2, :]) - np.mean(depth_norm[h//2:, :]))
            safety_msg.z = 2.0 
            self.target_pub.publish(safety_msg)

            if self.lantern_found:
                hunt_msg = Vector3()
                hunt_msg.x = self.lantern_x_err
                hunt_msg.y = self.lantern_area
                hunt_msg.z = self.lantern_y_err # Vertical Tracking Error
                self.target_pub.publish(hunt_msg)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = LanternTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
