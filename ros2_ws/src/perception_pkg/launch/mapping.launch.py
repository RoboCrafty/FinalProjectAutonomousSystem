from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node to convert Depth Image to PointCloud2 (Keep Original)
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
        
        # 2. Octomap Server (Keep Original Optimized 3D Settings)
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.6},              
                {'frame_id': 'world'},            
                {'base_frame_id': 'true_body'},   
                {'sensor_model.max_range': 40.0}, 
                {'occupancy_min_z': -50.0},        
                {'occupancy_max_z': 50.0}, 
                {'latch': True}
            ],
            remappings=[
                ('cloud_in', '/realsense/depth/points') 
            ],
            output='screen'
        ),

        # 3. NEW: Lantern Tracker Node (Added Perception Script)
        Node(
            package='perception_pkg',
            executable='lantern_tracker',
            name='lantern_tracker',
            output='screen'
        )
    ])
