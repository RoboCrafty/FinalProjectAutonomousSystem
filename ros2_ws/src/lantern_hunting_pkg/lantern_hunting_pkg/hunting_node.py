#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Transform
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class HunterNode(Node):
    def __init__(self):
        super().__init__('hunter_node')
        self.target_sub = self.create_subscription(Vector3, '/lantern/target_data', self.auto_steer_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'current_state', self.odom_callback, 10)
        self.traj_pub = self.create_publisher(MultiDOFJointTrajectory, '/command/trajectory', 10)
        
        self.state = "HUNTING" 
        self.tx, self.ty, self.tz = 0.0, 0.0, 0.0
        self.current_yaw = 3.14159
        self.start_time = self.get_clock().now()
        self.reach_time = None
        
        self.create_wall_timer(0.1, self.publish_loop)

    def odom_callback(self, msg):
        if self.tx == 0.0:
            self.tx, self.ty, self.tz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z

    def auto_steer_callback(self, msg):
        if (self.get_clock().now() - self.start_time).nanoseconds / 1e9 < 2.0: return
        if self.state != "HUNTING" or msg.z == 2.0: return # Skip if in exploration mode

        stop_threshold = 800000.0 #
        self.current_yaw -= msg.x * 0.001 # Centering

        if msg.y < stop_threshold:
            # Variable speed based on 70k area threshold
            hunt_speed = 0.06 if msg.y > 30000.0 else 0.12 
            self.tx += np.cos(self.current_yaw) * hunt_speed
            self.ty += np.sin(self.current_yaw) * hunt_speed

            # Diving condition based on 50k area threshold
            if msg.y > 30000.0:
                self.tz -= msg.z * 0.004
        else:
            self.state = "LOGGING"
            self.reach_time = self.get_clock().now()
            self.clearing_target_z = self.tz + 10.0

    def publish_loop(self):
        if self.state == "LOGGING":
            if (self.get_clock().now() - self.reach_time).nanoseconds / 1e9 >= 2.0:
                self.state = "CLEARING_ZONE"
                self.reach_time = self.get_clock().now()
        elif self.state == "CLEARING_ZONE":
            if self.tz < (self.clearing_target_z - 0.1):
                self.tz += 0.4 # Vertical Climb
                self.reach_time = self.get_clock().now()
            else:
                self.tx += np.cos(self.current_yaw) * 0.12 # Forward Burst
                self.ty += np.sin(self.current_yaw) * 0.12
                if (self.get_clock().now() - self.reach_time).nanoseconds / 1e9 >= 10.0:
                    self.state = "DONE"
        self.send_command()

    def send_command(self):
        msg = MultiDOFJointTrajectory()
        point = MultiDOFJointTrajectoryPoint()
        trans = Transform()
        trans.translation.x, trans.translation.y, trans.translation.z = self.tx, self.ty, self.tz
        trans.rotation.z, trans.rotation.w = np.sin(self.current_yaw/2), np.cos(self.current_yaw/2)
        point.transforms.append(trans)
        msg.points.append(point)
        self.traj_pub.publish(msg)

def main():
    rclpy.init(); node = HunterNode(); rclpy.spin(node); rclpy.shutdown()
