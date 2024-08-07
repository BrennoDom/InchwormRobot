from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    pkg_name = "flare_gazebo"

    # Get substitution for all arguments
    topic_prefix = str(LaunchConfiguration("topic_prefix").perform(context))
    camera_name = str(LaunchConfiguration("camera_name").perform(context))
    ros_gz_sim_args = str(LaunchConfiguration("ros_gz_sim_args").perform(context))

    camera_xacro_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", "environment_camera.xacro"]
    )
    
    camera1_model = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", camera_xacro_file, " topic_prefix:=", topic_prefix])
    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        Node(
            package="ros_gz_sim", 
            executable="create", 
            name="spawm_camera_gazebo",            
            arguments=[ "-name", camera_name, "-string", camera1_model] + ros_gz_sim_args.split(),
            output="screen",
        )
    ]


    return launch_descriptions

def generate_launch_description():
    """
    Generate list of all launch arguments that are declared for this launch script.
    """
    return LaunchDescription([
        # World for Ignition Gazebo
        DeclareLaunchArgument(
            "topic_prefix",
            default_value="camera1",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="env_camera",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "ros_gz_sim_args",
            default_value="-x 0 -y 0 -z 0.4 -R 0 -P 0 -Y 0",
            description="ros_gz_sim extra args",
        ),
        OpaqueFunction(function=launch_setup)
    ])
