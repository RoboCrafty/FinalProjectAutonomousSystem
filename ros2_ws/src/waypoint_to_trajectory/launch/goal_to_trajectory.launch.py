from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument (currently not used further, but kept for parity)
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value="firefly",
        description="Name of the MAV"
    )
    mav_name = LaunchConfiguration("mav_name")

    # Path to trajectory_config.yaml
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("waypoint_to_trajectory"),
        "config",
        "trajectory_config.yaml"
    ])

    # Trajectory planner node
    planner_node = Node(
        package="waypoint_to_trajectory",
        executable="waypoint_to_trajectory_node",   # ROS1: type="basic_waypoint_pkg"
        name="planner",
        output="screen",
        parameters=[trajectory_config]
    )

    # Trajectory sampler node
    sampler_node = Node(
        package="mav_trajectory_generation",
        executable="trajectory_sampler_node",
        name="sampler",
        output="screen",
        remappings=[
            ("path_segments_4D", "trajectory"),
        ],
    )

    return LaunchDescription([
        mav_name_arg,
        planner_node,
        sampler_node,
    ])
