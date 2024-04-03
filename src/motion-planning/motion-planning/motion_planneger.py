 from math import sin, cos, pi,atan2,tan
import math
import rclpy
import sys
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String , Int32MultiArray
from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from escalador_interfaces.srv import ChangeBase
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel


class StatePublisher(Node):

    flagBase    = False
    PrefixTopic = 'robot1'
    RobotURDF1 =''
    RobotURDF2 =''
    degree = pi / 180.0
    Base = Int8()
    J1 = 0.0 * degree
    J2 = 30.0 * degree
    J3 = 30.0 * degree
    J4 = 0.0 * degree
    J5 = 30.0 * degree
    J6 = 0.0 * degree
    flaginit = False
    i = 0

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        #self.create_subscription(Int32MultiArray,'/set_joints',self.joint_callback,qos_profile)
        #self. = self.create_publisher(JointState, 'joint_states', qos_profile)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def euler_from_quaternion(x, y, z, w):
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()