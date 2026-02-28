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
        
        # 2. NEW: Node to build the 3D Voxel Grid
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.2},  # Size of each voxel block (0.2 meters)
                {'frame_id': 'world'},  # The global map frame
                {'base_frame_id': 'true_body'}, # The drone's body frame
                {'sensor_model.max_range': 25.0} # Ignore points further than 8 meters
            ],
            remappings=[
                ('cloud_in', '/realsense/depth/points') # Feed the points we just made into Octomap
            ],
            output='screen'
        )
    ])