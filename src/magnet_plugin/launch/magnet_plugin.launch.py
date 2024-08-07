"""Visualisation of SDF model as world in Ignition Gazebo"""

from typing import List
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    gz_ros_bridge_file = LaunchConfiguration("gz_ros_bridge_file")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("gz_args", [" -v ", ign_verbosity, " -r ", world])],
        ),
    ]

    # ROS - IGN bridge
    # Gz - ROS Bridge
    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("magnet_plugin"), "config", gz_ros_bridge_file]
    )

    launch_descriptions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': gz_bridge_config,
            }],
            output='screen',
        )
    )

    return LaunchDescription(declared_arguments + launch_descriptions)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("magnet_plugin"), "worlds", "world.sdf"]),
            description="Name or filepath of world to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            'gz_ros_bridge_file',
            default_value="magnet_plugin_gz_ign_bridge.yaml",
            description='Parameter file to gz_ros_bridge pkg'
        ),
    ]
