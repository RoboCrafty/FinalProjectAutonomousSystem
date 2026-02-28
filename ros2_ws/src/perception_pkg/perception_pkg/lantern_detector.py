import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

# TF2 Imports
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

class LanternDetector(Node):
    def __init__(self):
        super().__init__('lantern_detector')
        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/Quadrotor/Sensors/SemanticCamera/camera_info',
            self.info_callback, 10)

        self.sem_sub = message_filters.Subscriber(self, Image, '/Quadrotor/Sensors/SemanticCamera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/realsense/depth/image')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sem_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('Node Started... Waiting for camera info.')

    def info_callback(self, msg):
        if self.fx is None:
            # We must access the array indices [0, 2, 4, 5] then cast to float
            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])
            self.fy = float(msg.k[4])
            self.cy = float(msg.k[5])
            self.get_logger().info(f'Loaded Intrinsics: fx={self.fx:.2f}, cx={self.cx:.2f}')

    def sync_callback(self, sem_msg, depth_msg):
        if self.fx is None: return 

        try:
            sem_img = self.bridge.imgmsg_to_cv2(sem_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough") 
            
            mask = cv2.inRange(sem_img, np.array([0, 200, 200]), np.array([50, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                if cv2.contourArea(cnt) > 100: 
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]) 
                        
                        # Parallax window
                        win = 6 
                        v_s, v_e = max(0, v-win), min(depth_img.shape[0], v+win)
                        u_s, u_e = max(0, u-win), min(depth_img.shape[1], u+win)
                        depth_region = depth_img[v_s:v_e, u_s:u_e]
                        
                        valid = depth_region[(depth_region > 0) & (depth_region < 10000)]
                        
                        if len(valid) > 0:
                            Z = float(np.min(valid)) / 1000.0  
                            X = (u - self.cx) * Z / self.fx
                            Y = (v - self.cy) * Z / self.fy
                            
                            p_cam = PointStamped()
                            # Tell TF to just use the most recent position it has!
                            p_cam.header.stamp = rclpy.time.Time().to_msg() 
                            p_cam.header.frame_id = 'camera'
                            p_cam.point.x, p_cam.point.y, p_cam.point.z = X, Y, Z

                            try:
                                # Transform to world
                                p_world = self.tf_buffer.transform(p_cam, 'world')
                                self.get_logger().info(f'Lantern: X={p_world.point.x:.2f} Y={p_world.point.y:.2f} Z={p_world.point.z:.2f}')
                            except Exception as e:
                                self.get_logger().warn(f'TF: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LanternDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()