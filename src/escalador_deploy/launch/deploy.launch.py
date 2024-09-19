from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit,OnExecutionComplete,OnProcessStart
import xacro
gui = LaunchConfiguration("gui")

def generate_launch_description():
    package_name = "escalador_deploy"
    description_file = LaunchConfiguration("description_file")
    declared_arguments = []

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
    prefix = LaunchConfiguration("prefix")

    controller_config = os.path.join(
        get_package_share_directory(
            "escalador_deploy"), "controllers", "controllers.yaml"
    )
    
    share_dir = get_package_share_directory('escalador_deploy')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

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
    kinematics = Node(
        package ='kdl_kin_escalador',
        executable="ik_kinematics_node",
        name="ik_kinematics_node",
        output="screen"
    )
    robot_controller_node = Node(
        
	package="controller_manager",
	executable="ros2_control_node",
        parameters=[robot_description,controller_config],
	output="both",
    )
    robot_joint_state_broadcaster = Node(
    	 
         package="controller_manager",
         executable="spawner",
         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
         output="log",
    
    )
    robot_controller_position = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "-c", "/controller_manager"],
            output="log",
    )

    robot_controller_velocity = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="log",
    )
    BASE1_CONTROLLER = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["BASE1_CONTROLLER", "-c", "/controller_manager"],
            output="log",
    )
    BASE2_CONTROLLER = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["BASE2_CONTROLLER", "-c", "/controller_manager"],
            output="log",
    )
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
        output="log",
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[robot_joint_state_broadcaster],
                )
            ]
        )
    )
    delay_robot_controller_position = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[robot_controller_position],
                )
            ]
        )
    )
    delay_controller_velocity = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[robot_controller_velocity],
                )
            ]
        )
    )
    delay_BASE1_trajectory = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[BASE1_CONTROLLER],
                )
            ]
        )
    )
    delay_BASE2_trajectory = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[BASE2_CONTROLLER],
                )
            ]
        )
    )
    delay_gpio_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_node,
            on_start=[
                TimerAction(
                period=10.0,
                actions=[gpio_controller_spawner],
                )
            ]
        )
    )


    nodes = [
        state_1,
        rviz_node,
        default_base,
        state_2,
        state_publisher,
        robot_controller_node,
        delay_joint_state_broadcaster,
        delay_robot_controller_position,
        delay_controller_velocity,
        delay_BASE1_trajectory,
        delay_BASE2_trajectory,
        delay_gpio_controller_spawner

    ]
    return LaunchDescription(declared_arguments + nodes)
