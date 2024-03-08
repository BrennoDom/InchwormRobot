from math import sin, cos, pi
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from escalador_interfaces.srv import ChangeBase

class StatePublisher(Node):


    PrefixTopic = 'robot1'
    RobotURDF1 =''
    RobotURDF2 =''
    degree = pi / 180.0
    Base = 0
    J1 = 0.0 * degree
    J2 = 60.0 * degree
    J3 = 60.0 * degree
    J4 = 0.0 * degree
    J5 = 60.0 * degree
    J6 = 0.0 * degree

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        qos_profile_URDF = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,depth=1)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.create_subscription(String,'robot1/robot_description',self.robot1_callback,qos_profile_URDF)
        self.create_subscription(String,'robot2/robot_description',self.robot2_callback,qos_profile_URDF)
        self.robot_pub = self.create_publisher(String, 'robot_description', qos_profile_URDF)
        self.timer_ = self.create_timer(0.1, self.publish_joint)
        self.create_service(ChangeBase,'SrvChangeBase',self.ChangeBaseCallback)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        

        

        # robot state


        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
               

        except KeyboardInterrupt:
            pass
    
    def ChangeBaseCallback(self,request,response):

        string_robot = String()
        if request.change is False:
            string_robot.data = self.RobotURDF1
            response.basename = 'Base 0'
            self.PrefixTopic = 'robot1'
            self.Base = 0
        else:
            string_robot.data = self.RobotURDF2
            response.basename = 'Base 1'
            self.PrefixTopic = 'robot2'
            self.Base = 1
        self.get_logger().info('Incoming request\n%s' % (request.change))  
        
        self.robot_pub.publish(string_robot)

        return response
    
    def publish_joint(self):
        
        odom_trans = TransformStamped()

        now = self.get_clock().now()
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['J1', 'J2', 'J3','J4', 'J5', 'J6']
        joint_state.position = [self.J1, self.J2, self.J3, self.J4, self.J5, self.J6]
        match self.Base:
            case 0:
                odom_trans.header.frame_id = 'world'
                odom_trans.child_frame_id = 'robot1/LINK_1'


                        # send the joint state and transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0
                odom_trans.transform.rotation = \
                euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

            case 1:
                odom_trans.header.frame_id = 'world'
                odom_trans.child_frame_id = 'robot2/LINK_7'

                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0
                odom_trans.transform.rotation = \
                euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
            
        self.joint_pub.publish(joint_state)
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

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()