from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "flare_gazebo"
    launch_dir = ThisLaunchFileDir()
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "flare_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments = {"world":"conceitual_flare_world.sdf", "ign_verbosity":"3"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "environment_camera.launch.py",
                    ]
                )
            ),
            launch_arguments = {"topic_prefix":"camera_env", "camera_name":"camera_env", "ros_gz_sim_args":"-x 18.3 -y 12.5 -z 3.0 -Y -1.0"}.items(),
        ),
    ])
