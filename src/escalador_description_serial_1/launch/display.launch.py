from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    share_dir = get_package_share_directory('escalador_description_serial_1')

    xacro_file = os.path.join(share_dir, 'urdf', 'escalador_serial_1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        namespace='robot1',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_1',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )
    joint_state_publisher_node = Node(
        namespace='robot1',
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_1'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])