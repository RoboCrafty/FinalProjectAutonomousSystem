#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # Import for the error message
from cv_bridge import CvBridge
import cv2
import numpy as np

class LanternTracker(Node):
    def __init__(self):
        super().__init__('lantern_tracker')
        
        # 1. Subscriber: Listens to the Semantic Camera feed
        self.subscription = self.create_subscription(
            Image,
            '/Quadrotor/Sensors/SemanticCamera/image_raw',
            self.image_callback,
            10)
        
        # 2. Publisher: Sends the "Error" to the C++ Auto-Pilot
        self.error_pub = self.create_publisher(Float32, '/lantern/horizontal_error', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Lantern Tracker Online. Publishing error to /lantern/horizontal_error")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV color space for stable color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the Semantic Yellow range based on your feed
        # These values target that bright yellow lantern in your screenshot
        lower_yellow = np.array([25, 150, 150])
        upper_yellow = np.array([35, 255, 255])
        
        # Create a mask (Black and White image where Yellow is White)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate moments to find the center
        moments = cv2.moments(mask)
        if moments["m00"] > 50: # Threshold: ignore tiny noise
            cx = int(moments["m10"] / moments["m00"])
            
            # --- THE NAVIGATION LOGIC ---
            # Most drone cameras in this sim are 320 pixels wide.
            # Center is 160. 
            # If cx = 110, error is -50 (Lantern is to the left).
            image_center_x = 160.0 
            error_val = float(cx - image_center_x)
            
            # Publish the error for the C++ node
            error_msg = Float32()
            error_msg.data = error_val
            self.error_pub.publish(error_msg)
            
            self.get_logger().info(f"Lantern @ x={cx} | Sending Error: {error_val}")
        else:
            # If no lantern is seen, we don't send an error 
            # (or we could send a specific 'search' value)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = LanternTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
