from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.4},  # Improved for cave geometry
                {'frame_id': 'world'},
                {'base_frame_id': 'true_body'},
                {'sensor_model.max_range': 25.0},
                {'latch': True}
            ],
            remappings=[('cloud_in', '/realsense/depth/points')],
            output='screen'
        )
    ])