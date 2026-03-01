from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Convert Depth Image to PointCloud2 (same as octomap launch)
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

        # 2. RTAB-Map node: build a 3D map and publish an OctoMap-compatible topic
        Node(
            package='rtabmap_launch',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                {'frame_id': 'world'},
                # Request RTAB-Map to publish an octomap (may require rtabmap_ros installed)
                {'PublishOctomap': True},
                {'Octomap/OccupancyThreshold': 0.5},
            ],
            remappings=[
                # Make sure RTAB-Map listens to the same pointcloud topic
                ('/camera/depth/points', '/realsense/depth/points'),
                # Remap RTAB-Map's octomap output to the expected topic
                ('/rtabmap/octomap', '/octomap_binary'),
                ('/octomap', '/octomap_binary')
            ],
            output='screen'
        )
    ])
