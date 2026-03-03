import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the perception mapping launch
    perception_launch_path = os.path.join(
        get_package_share_directory('perception_pkg'),
        'launch',
        'mapping.launch.py'
    )

    return LaunchDescription([
        # 1. Start Perception/Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch_path)
        ),

        # 2. Frontier Explorer
        Node(
            package='exploration_pkg',
            executable='frontier_explorer_node',
            name='frontier_explorer',
            output='screen'
        ),

        # 3. Trajectory Generator
        Node(
            package='trajectory_generator_pkg',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='log'
        ),

        # 4. Lantern Hunting
        Node(
            package='lantern_hunting_pkg',
            executable='hunting_node',
            name='hunter_node',
            output='screen'
        ),

        # 5. State Machine
        Node(
            package='decision_making_pkg',
            executable='state_machine_node',
            name='state_machine',
            output='screen'
        ),
    ])