from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    pkg_name = "escalador_description_serial_1"

    # Launch Arguments
    run_control = str(LaunchConfiguration('run_control').perform(context))

    controller_type = str(LaunchConfiguration(
        'controller_type').perform(context))

    robot_model_name = str(LaunchConfiguration(
        'robot_model_name').perform(context))

    robot_gazebo_args = str(LaunchConfiguration(
        'robot_gazebo_args').perform(context))

    gz_ros_bridge_file_name = str(LaunchConfiguration(
        'gz_ros_bridge_file').perform(context))

    
    enable_magnetic_gripper = str(LaunchConfiguration(
        'enable_magnetic_gripper').perform(context))
    
    # Full path to robot xacro file
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", robot_model_name]
    )

    robot_model = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]),
         ' ', xacro_file, " run_control:=", run_control, " prefix:=", "robot1/"]
    )

    # Launch Gazebo world #
    launch_flare = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("flare_gazebo"),
                    "launch",
                    "flare_and_camera.launch.py",
                ]
            )
        )
    )

    # Insert robot model in simulation
    spawn_robot_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawm_robot_gazebo',
        arguments=['-name', 'robot_serial_1', '-string',
                   robot_model] + robot_gazebo_args.split(),
        output='screen',
    )

    # Start simulation with magnetic gripper on
    # ros2 topic pub /tail/switch std_msgs/msg/Bool 'data: false' --once
    
    enable_magnet = ExecuteProcess(
        cmd=[[
            'ros2 topic pub ',
            '/tail/switch ',
            'std_msgs/msg/Bool ',
            '"data: true" --once'
        ]],
        shell=True
    )
    

    set_gz_ros_control = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[[
                'ros2 param set ',
                '/gz_ros2_control ',
                'use_sim_time ',
                'True'
            ]],
            shell=True
        )],
    )
    
    load_magnet_delay_node = TimerAction(
        period=5.0,
        actions=[enable_magnet],
    )
    
    # ROS - IGN bridge
    # Gz - ROS Bridge
    robot_gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", gz_ros_bridge_file_name]
    )

    use_sim_time = {'use_sim_time': True}
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': robot_gz_bridge_config}, use_sim_time],
        output='screen',
    )

    params = {'robot_description': robot_model, 'use_sim_time': True}
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Set controllers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[use_sim_time],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_type],
        output="screen",
        parameters=[use_sim_time],
    )

    node_2_run = [launch_flare,
                  spawn_robot_model,
                  ros_gz_bridge_node,
                  set_gz_ros_control,
                  ]

    if run_control.lower() == 'ros2_control':
        node_2_run.append(robot_state_node)
        node_2_run.append(load_joint_state_broadcaster)
        node_2_run.append(load_arm_controller)
    
    if enable_magnetic_gripper.lower() == 'true':
        if run_control.lower() == 'ros2_control':
            node_2_run.append(RegisterEventHandler(event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster, on_exit=enable_magnet)))
        else:
            node_2_run.append(load_magnet_delay_node)
    
    return node_2_run


def generate_launch_description():
    # Args
    run_control = DeclareLaunchArgument(
        'run_control',
        default_value='ros2_control',
        description='Available options are: \'ros2_control\', \'moveit_control\', and \'false\'')

    controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='arm_controller',
        description='If run_ros2_control is ros2_control, defines the type of controller: arm_controller or joint_pos_controller')

    robot_model_name_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value="escalador_serial_1_gazebo.urdf.xacro",
        description='Robot model name file to load')

    robot_gazebo_args = DeclareLaunchArgument(
        'robot_gazebo_args',
        default_value="-x 19.5 -y 11.072525 -z 3.0 -Y 0.0 -R -1.57 -P -1.57 -Y -0.1",
        description='robot extra args to send to gazebo')

    gz_ros_bridge_file_name_arg = DeclareLaunchArgument(
        'gz_ros_bridge_file',
        default_value="mvp_6j_gz_ign_bridge.yaml",
        description='Parameter file to gz_ros_bridge pkg')
    
    enable_magnetic_gripper_arg = DeclareLaunchArgument(
        'enable_magnetic_gripper',
        default_value='true',
        description='If true turn magnetic force on.')

    launch_args = [run_control,
                   controller_type,
                   robot_model_name_arg,
                   robot_gazebo_args,
                   gz_ros_bridge_file_name_arg,
                   enable_magnetic_gripper_arg
                   ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])