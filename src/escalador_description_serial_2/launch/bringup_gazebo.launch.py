import os
import os.path
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments

    # Launching ign_gazebo.launch.py is comsumed more than 1 minute somehow...
    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "--spin-time", "120",
             "joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen"
    )

    velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "configure",
             "velocity_controller"],
        output="screen"
    )
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="escalador_description_serial_2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="escalador_serial_2.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
        )
    )

    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("escalador_description_serial_2"), "urdf", description_file]
            ),
            " ",
            "prefix:=robot2/",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        namespace='robot2',
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    state_publisher = Node(
        namespace='robot2',
        package="escalador_description_serial_2",
        executable="state_publisher",
        name="state_publisher",
        output="screen"
    )
    nodes = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_controller,
                on_exit=[velocity_controller],
            )
        ),
        robot_state_publisher_node,
        state_publisher
    ]

    return LaunchDescription(declared_arguments + nodes)
