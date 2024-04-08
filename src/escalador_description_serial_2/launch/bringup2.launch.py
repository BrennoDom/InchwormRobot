import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments

    controller_config = os.path.join(
        get_package_share_directory(
            "escalador_description_serial_2"), "controllers", "controllers.yaml"
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

    robot_controller_node = Node(
        
	package="controller_manager",
	executable="ros2_control_node",
        parameters=[robot_description, controller_config],
	output="screen",
    )
    robot_joint_state_broadcaster = Node(
    	 
         package="controller_manager",
         executable="spawner",
         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
         output="screen",
    
    )

    robot_controller_velocity = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
    )
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
        robot_state_publisher_node,
        state_publisher,
        robot_controller_node,
        robot_joint_state_broadcaster,
        robot_controller_velocity
    ]

    return LaunchDescription(declared_arguments + nodes)
