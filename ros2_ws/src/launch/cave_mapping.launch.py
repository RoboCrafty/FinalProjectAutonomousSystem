from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': 'base_link',          # Your drone's center
                'subscribe_depth': True,
                'subscribe_odom_info': False,
                'approx_sync': True,             # Crucial for simulation jitter
                'use_sim_time': True,
                'queue_size': 20,
                
                # Expert Tuning for Caves
                'RGBD/AngularUpdate': '0.01',    # Update map even on small turns
                'RGBD/LinearUpdate': '0.01',     # Update map on small movements
                'Grid/RangeMax': '5.0',          # Ignore depth noise past 5m
                'Grid/FromDepth': 'true',        # Build the floor map from depth
                'Reg/Force3DoF': 'false',        # Drones move in 6DoF (XYZ + RPY)
                'Mem/IncrementalMemory': 'true', # true = Mapping, false = Localization only
            }],
            remappings=[
                ('rgb/image', '/realsense/rgb/left_image_raw'),
                ('depth/image', '/realsense/depth/image'),
                ('rgb/camera_info', '/realsense/rgb/left_image_info'),
                ('imu', '/Quadrotor/Sensors/IMU'),
                ('odom', '/current_state_est'), # Using your sim's estimate as a base
            ]
        ),
        
        # Optional: Start the RTAB-Map GUI to see the 3D loop closures
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{'subscribe_depth': True, 'use_sim_time': True}]
        ),
    ])