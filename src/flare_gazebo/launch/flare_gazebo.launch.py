"""Visualisation of SDF model as world in Ignition Gazebo"""
import os

from typing import List
from launch import LaunchDescription #
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    ign_verbosity = LaunchConfiguration("ign_verbosity")

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
            launch_arguments=[("gz_args", [" -v ", ign_verbosity, " ", world])],
        ),
    ]


    return LaunchDescription(declared_arguments + launch_descriptions)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """
    return [
        # World for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="conceitual_flare_world.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
    ]