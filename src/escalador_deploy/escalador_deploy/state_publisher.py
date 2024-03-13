from math import sin, cos, pi
import math
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformStamped
from escalador_interfaces.srv import ChangeBase

class StatePublisher(Node):


    PrefixTopic = 'robot1'
    RobotURDF1 =''
    RobotURDF2 =''
    degree = pi / 180.0
    Base = 1
    J1 = 30.0 * degree
    J2 = 60.0 * degree
    J3 = 60.0 * degree
    J4 = 40.0 * degree
    J5 = 60.0 * degree
    J6 = 30.0 * degree

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
       
        self.buffertf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffertf,self)

        

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
   

        #odom_base_2 = self.buffertf.lookup_transform('robot2/LINK_1','robot2/LINK_7',rclpy.time.Time())
        #self.get_logger().info('%s' % (odom_base_2)) 


        now = self.get_clock().now()
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        odom_base_1 = self.buffertf.lookup_transform('robot1/LINK_1','robot1/end-effector',rclpy.time.Time())
        odom_base_2 = self.buffertf.lookup_transform('robot2/LINK_7','robot2/end-effector',rclpy.time.Time())
        #self.get_logger().info('%s' % (odom_base_1)) 
        joint_state.name = ['J1', 'J2', 'J3','J4', 'J5', 'J6']
        joint_state.position = [self.J1, self.J2, self.J3, self.J4, self.J5, self.J6]
        match self.Base:
            case 0:


                odom_trans.header.frame_id = 'odom'
                odom_trans.child_frame_id = 'robot1/LINK_1'


                        # send the joint state and transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0
                odom_trans.transform.rotation = \
                euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

            case 1:


                odom_trans.header.frame_id = 'odom'
                odom_trans.child_frame_id = 'robot2/LINK_7'
                
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation = odom_base_1.transform.translation
                angles = euler_from_quaternion(odom_base_1.transform.rotation.x,odom_base_1.transform.rotation.y,odom_base_1.transform.rotation.z,odom_base_1.transform.rotation.w)
                odom_trans.transform.rotation = \
                euler_to_quaternion(-angles[0], angles[1]+3.14, angles[2]) # roll,pitch,yaw
        
       # odom_base_1 = self.buffertf.lookup_transform('robot1/LINK_7','robot2/dummy_base',rclpy.time.Time())
        #self.get_logger().info('%s' % ()) 
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