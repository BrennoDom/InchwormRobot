from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import xacro
import os
from escalador_interfaces.srv import ChangeBase                                                           # CHANGE

import rclpy
from rclpy.node import Node

share_dir_1 = get_package_share_directory('escalador_description_serial_1')
share_dir_2 = get_package_share_directory('escalador_description_serial_2')

xacro_file_1 = os.path.join(share_dir_1, 'urdf', 'escalador_serial_1.xacro')
xacro_file_2 = os.path.join(share_dir_2, 'urdf', 'escalador_serial_2.xacro')
robot_description_config_1 = xacro.process_file(xacro_file_1)
robot_description_config_2 = xacro.process_file(xacro_file_2)
robot_urdf_1 = robot_description_config_1.toxml()
robot_urdf_2 = robot_description_config_2.toxml()

class ChangeBaseService(Node):

    def __init__(self):
        super().__init__('chbase_service')
        self.srv = self.create_service(ChangeBase, 'ChangeBase', self.add_changebase_callback)       # CHANGE

    def add_changebase_callback(self, request, response):

        if request.change is False:
            response.basename = 'Base 0'
            

        else:
            response.basename = 'Base 1'
        self.get_logger().info('Incoming request\n%s' % (request.change))  # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    chbase_service = ChangeBaseService()

    rclpy.spin(chbase_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()