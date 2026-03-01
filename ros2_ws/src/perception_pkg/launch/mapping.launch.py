from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node to convert Depth Image to PointCloud2
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='point_cloud_xyz_node',
            remappings=[
                ('image_rect', '/realsense/depth/image'),
                ('camera_info', '/realsense/depth/camera_info'),
                ('points', '/realsense/depth/points')
            ],
            output='screen'
        ),
        
        # 2. THE ONLY Octomap Server (Optimized for 3D CPU Performance)
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.4},              # Sweet spot for 3D C++ Raycasting
                {'frame_id': 'world'},            # The global map frame
                {'base_frame_id': 'true_body'},   # The drone's body frame
                {'sensor_model.max_range': 40.0}, # See far enough for safe leaps, but ignore the void
                {'occupancy_min_z': -50.0},        # Keeps the RViz 2D map visualization clean
                {'occupancy_max_z': 50.0}, 
                {'latch': True}
            ],
            remappings=[
                ('cloud_in', '/realsense/depth/points') 
            ],
            output='screen'
        )
    ])