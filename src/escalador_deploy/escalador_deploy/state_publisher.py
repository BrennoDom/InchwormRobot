from math import sin, cos, pi,atan2,tan
import math
import rclpy
import sys
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_msgs.msg import String , Int32MultiArray, Float64MultiArray
from std_msgs.msg import Int8
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
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
from builtin_interfaces.msg import Duration

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
        qos_profile_URDF = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,depth=1)
        #self.create_subscription(Int32MultiArray,'/set_joints',self.joint_callback,qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.create_subscription(String,'robot1/robot_description',self.robot1_callback,qos_profile_URDF)
        self.create_subscription(String,'robot2/robot_description',self.robot2_callback,qos_profile_URDF)
        self.robot_pub = self.create_publisher(String, 'robot_description', qos_profile_URDF)
        self.base_pub = self.create_publisher(Int8, 'act_base', 10)
        self.timer_ = self.create_timer(0.03, self.publish_joint)
        self.create_service(ChangeBase,'SrvChangeBase',self.ChangeBaseCallback)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
       
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        # robot state


        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
               

        except KeyboardInterrupt:
            pass

    def joint_callback(self, msg):
       #self.get_logger().info(str(msg.position))
       self.position =[0,0,0,0,0,0]

       self.J1 = msg.data[0]* self.degree
       self.J2 = msg.data[1]* self.degree
       self.J3 = msg.data[2]* self.degree
       self.J4 = msg.data[3]* self.degree
       self.J5 = msg.data[4]* self.degree
       self.J6 = msg.data[5]* self.degree
    def ChangeBaseCallback(self,request,response):

        self.flagBase = False;
        string_robot = String()
        if request.change is False:

            string_robot.data = self.RobotURDF1
            response.basename = 'Base 0'
            self.PrefixTopic = 'robot1'
            self.Base.data = 0
        else:
            string_robot.data = self.RobotURDF2

            response.basename = 'Base 1'
            self.PrefixTopic = 'robot2'
            self.Base.data = 1
        
        self.get_logger().info('Incoming request\n%s' % (request.change))  
            
        self.robot_pub.publish(string_robot)

        return response
    
    def publish_joint(self):
      
        odom_trans = TransformStamped()
        now = self.get_clock().now()
        joint_state = JointState()
   

        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'actual_odom'
                
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = 0.0
        odom_trans.transform.translation.y = 0.0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = \
        euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

      #  self.joint_pub.publish(joint_state)
        self.base_pub.publish(self.Base)


                        

        self.broadcaster.sendTransform(odom_trans)

    def robot1_callback(self,msg):
        self.RobotURDF1 = msg.data
    def robot2_callback(self,msg):
        self.RobotURDF2 = msg.data

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
