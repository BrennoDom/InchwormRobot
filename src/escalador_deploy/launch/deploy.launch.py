from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('escalador_deploy')

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    state_publisher = Node(
            package='escalador_deploy',
            executable='state_publisher',
            name="state_publisher",
            output="screen"
            
        )


    return LaunchDescription([
        gui_arg,
        rviz_node,
        state_publisher
        
    ])