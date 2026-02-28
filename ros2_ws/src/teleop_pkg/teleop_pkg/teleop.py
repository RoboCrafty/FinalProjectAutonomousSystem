import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
import math
import sys
import select
import termios
import tty
import threading

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(MultiDOFJointTrajectory, '/command/trajectory', 10)
        
        # CHANGED: Now listening to /true_pose to get the initial coordinates
        self.pose_sub = self.create_subscription(PoseStamped, '/true_pose', self.pose_callback, 10)
        
        self.initialized = False
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_vyaw = 0.0
        
        self.max_speed = 4.0  
        self.max_yaw_rate = 1.0 
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.game_loop)

    def pose_callback(self, msg):
        if not self.initialized:
            self.target_x = msg.pose.position.x
            self.target_y = msg.pose.position.y
            self.target_z = msg.pose.position.z
            q = msg.pose.orientation
            self.target_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            self.initialized = True
            
            # Print to terminal so you know it connected!
            print('\n[SUCCESS] Flight Computer Initialized at X:{:.2f} Y:{:.2f} Z:{:.2f}. Ready for takeoff!\n'.format(
                self.target_x, self.target_y, self.target_z))

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def game_loop(self):
        if not self.initialized:
            return

        # 1. Integrate position smoothly based on velocity
        global_vx = self.cmd_vx * math.cos(self.target_yaw) - self.cmd_vy * math.sin(self.target_yaw)
        global_vy = self.cmd_vx * math.sin(self.target_yaw) + self.cmd_vy * math.cos(self.target_yaw)
        
        self.target_x += global_vx * self.dt
        self.target_y += global_vy * self.dt
        self.target_z += self.cmd_vz * self.dt
        self.target_yaw += self.cmd_vyaw * self.dt

        # 2. Build the Trajectory Message
        msg = MultiDOFJointTrajectory()
        
        # CHANGED: Added the header to match your working command
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["base_link"]
        
        point = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = self.target_x
        transform.translation.y = self.target_y
        transform.translation.z = self.target_z
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.target_yaw)
        transform.rotation.x = qx
        transform.rotation.y = qy
        transform.rotation.z = qz
        transform.rotation.w = qw
        point.transforms.append(transform)
        
        vel = Twist()
        vel.linear.x = global_vx
        vel.linear.y = global_vy
        vel.linear.z = self.cmd_vz
        point.velocities.append(vel) 
        
        point.accelerations.append(Twist())
        
        # CHANGED: Added a small time_from_start just in case the controller requires it
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)
        
        msg.points.append(point)
        self.publisher_.publish(msg)

def key_listener(node):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    
    try:
        while rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                
                if key == 'w': node.cmd_vx = node.max_speed
                elif key == 's': node.cmd_vx = -node.max_speed
                elif key == 'a': node.cmd_vy = node.max_speed
                elif key == 'd': node.cmd_vy = -node.max_speed
                elif key == ' ': node.cmd_vz = node.max_speed
                elif key == 'z' or key == 'Z': node.cmd_vz = -node.max_speed
                elif key == 'q': node.cmd_vyaw = node.max_yaw_rate
                elif key == 'e': node.cmd_vyaw = -node.max_yaw_rate
                elif key == '\x03': # CTRL-C
                    break
            else:
                node.cmd_vx = 0.0
                node.cmd_vy = 0.0
                node.cmd_vz = 0.0
                node.cmd_vyaw = 0.0
                
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    print("""
    ===================================
    🚁 HELICOPTER FLIGHT CONTROLS 🚁
    ===================================
    W / S : Pitch Forward / Backward
    A / D : Roll Left / Right
    Space : Ascend (Up)
      Z   : Descend (Down) Note: Capital Z
    Q / E : Yaw Left / Right
    
    Waiting for position data from drone...
    """)
    
    thread = threading.Thread(target=key_listener, args=(node,))
    thread.daemon = True
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()