#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3  # Pack Error and Area into one msg
from cv_bridge import CvBridge
import cv2
import numpy as np

class LanternTracker(Node):
    def __init__(self):
        super().__init__('lantern_tracker')
        
        # Subscriber: Listens to the Semantic Camera feed
        self.subscription = self.create_subscription(
            Image,
            '/Quadrotor/Sensors/SemanticCamera/image_raw',
            self.image_callback,
            10)
        
        # Publisher: Sends Vector3 where x=Horizontal Error, y=Area, z=Visibility
        self.target_pub = self.create_publisher(Vector3, '/lantern/target_data', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Phase 1 Tracker Online. Sending Error and Area.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Targets the bright yellow lantern color
        lower_yellow = np.array([25, 150, 150])
        upper_yellow = np.array([35, 255, 255])
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        moments = cv2.moments(mask)
        
        target_msg = Vector3()

        if moments["m00"] > 50: # If lantern is visible
            cx = int(moments["m10"] / moments["m00"])
            area = moments["m00"] 
            
            # x=Horizontal Error, y=Area, z=Visibility Flag
            target_msg.x = float(cx - 160.0) 
            target_msg.y = float(area)       
            target_msg.z = 1.0               
            
            self.get_logger().info(f"Tracking - Area: {area:.0f} | Error: {target_msg.x:.1f}")
        else:
            target_msg.z = 0.0 # Flag: Nothing found

        self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LanternTracker()
    rclpy.spin(node)
    rclpy.shutdown()

# This part ensures the node actually starts when you run the file
if __name__ == '__main__':
    main()
