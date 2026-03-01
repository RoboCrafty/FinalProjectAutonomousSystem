import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters  # Keeps time synchronized

class LanternDetector(Node):
    def __init__(self):
        super().__init__('lantern_detector')
        
        self.bridge = CvBridge()
        
        # Storage for Camera Intrinsics (focal length, optical center)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # 1. Subscribe to Camera Info ONCE to get intrinsics
        # We use the Semantic Camera info to match our pixel coordinates
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/Quadrotor/Sensors/SemanticCamera/camera_info',
            self.info_callback,
            10)

        # 2. Setup Synchronized Subscribers
        # We subscribe to both color (semantic) and depth
        self.sem_sub = message_filters.Subscriber(self, Image, '/Quadrotor/Sensors/SemanticCamera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/realsense/depth/image')
        
        # Sync Policy: Wait until we have a message from BOTH topics with close timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sem_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1 # Allow 0.1s delay between messages
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info('Lantern Detector Node Started... Waiting for camera info.')

    def info_callback(self, msg):
        """ Runs once to save camera settings (focal length, center). """
        if self.fx is None:
            # K matrix is [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.get_logger().info(f'Loaded Intrinsics: fx={self.fx:.2f}, cx={self.cx:.2f}')

    def sync_callback(self, sem_msg, depth_msg):
        """ Runs only when BOTH images arrive together. """
        if self.fx is None:
            return # Cannot calculate 3D without intrinsics

        try:
            # 1. Convert Images
            sem_img = self.bridge.imgmsg_to_cv2(sem_msg, "bgr8")
            # Depth often comes as 32-bit float (meters) or 16-bit int (mm)
            # ROS "32FC1" means 32-bit Float, Channel 1.
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
            
            # 2. Find Lantern Center (u, v)
            lower_yellow = np.array([0, 200, 200])
            upper_yellow = np.array([50, 255, 255])
            mask = cv2.inRange(sem_img, lower_yellow, upper_yellow)
            
            # Use Moments to find the center of the blob
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100: # Filter small noise
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        u = int(M["m10"] / M["m00"]) # Center Column
                        v = int(M["m01"] / M["m00"]) # Center Row
                        
                        # 3. Read Depth (Z) at that pixel
                        # Check bounds first
                        if v < depth_img.shape[0] and u < depth_img.shape[1]:
                            Z = depth_img[v, u] / 1000.0  # millimeters to meters
                            
                            # 4. Apply Pinhole Camera Math
                            if np.isfinite(Z) and Z > 0:
                                X = (u - self.cx) * Z / self.fx
                                Y = (v - self.cy) * Z / self.fy
                                
                                self.get_logger().info(
                                    f'Lantern Found! Rel Coords: X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m'
                                )

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LanternDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()