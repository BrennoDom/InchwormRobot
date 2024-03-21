from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

gui = LaunchConfiguration("gui")

def generate_launch_description():
    share_dir = get_package_share_directory('escalador_deploy')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    state_1 = ExecuteProcess(
            cmd=[[
            'ros2 launch ',
            PathJoinSubstitution(
                [
                    FindPackageShare("escalador_description_serial_1"),
                    "launch",
                    "bringup1.launch.py",
                ]
            ),
        ]],
        shell=True
    )
    state_2 = ExecuteProcess(
            cmd=[[
            'ros2 launch ',
            PathJoinSubstitution(
                [
                    FindPackageShare("escalador_description_serial_2"),
                    "launch",
                    "bringup2.launch.py",
                ]
            ),
        ]],
        shell=True
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    default_base = ExecuteProcess(
            cmd=[[
            'ros2 service call ',
            '/SrvChangeBase ',
            'escalador_interfaces/srv/ChangeBase ',
            '"{change: False}"'
        ]],
        shell=True
    )
    state_publisher = Node(
            package='escalador_deploy',
            executable='state_publisher',
            name="state_publisher",
            output="screen"
            
        )


    return LaunchDescription([
        state_1,
        rviz_node,
        state_publisher,
        default_base,
        state_2
        
    ])