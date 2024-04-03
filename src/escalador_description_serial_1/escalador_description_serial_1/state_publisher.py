from math import sin, cos, pi, tan
import math
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from tf_transformations import euler_from_quaternion

class StatePublisher(Node):

    J1 = 0.
    J2 = 0.
    J3 = 0.
    J4 = 0.
    J5 = 0.
    J6 = 0.0
    Base = Int8()
    index = 0
    snapTrans = [0.0, 0.0,0.0]
    snapRot = [0,0,0,1] 
    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.timer_ = self.create_timer(0.03, self.publish_joint)
        self.get_logger().info("{0} started".format(self.nodeName))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.create_subscription(JointState,'/joint_states',self.joint_callback,qos_profile)
        self.create_subscription(Int8,'/act_base',self.base_callback,10)
        degree = pi / 180.0
        loop_rate = self.create_rate(30)

    def base_callback(self,msg):
        self.Base.data = msg.data


    def joint_callback(self, msg):
       #self.get_logger().info(str(msg.position))
       self.position =[0,0,0,0,0,0]

       self.J1 = msg.position[0]
       self.J2 = msg.position[1]
       self.J3 = msg.position[2]
       self.J4 = msg.position[3]
       self.J5 = msg.position[4]
       self.J6 = msg.position[5]

    def publish_joint(self):

        Trans = [0.0, 0.0,0.0]
        Rot = \
        euler_to_quaternion(0, 0, 0) 
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'actual_odom'
        odom_trans.child_frame_id = 'odom1'
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['J1', 'J2', 'J3','J4', 'J5', 'J6']
        joint_state.position = [self.J1, self.J2, self.J3, self.J4, self.J5, self.J6]
        self.joint_pub.publish(joint_state)



        match self.Base.data:
            case 0:
                try:
                    odom_base_1 = self.tf_buffer.lookup_transform('actual_odom','robot2/end-dummy',rclpy.time.Time())
                except TransformException as ex:
                    odom_trans.header.stamp = now.to_msg()
                    odom_trans.transform.translation.x = Trans[0]
                    odom_trans.transform.translation.y = Trans[1]
                    odom_trans.transform.translation.z = Trans[2]
                    odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 0) 
                    self.broadcaster.sendTransform(odom_trans)
                    return
                else:
                    match self.index:
                        
                        case 0:
                            self.snapTrans[0] = odom_base_1.transform.translation.x
                            self.snapTrans[1] = odom_base_1.transform.translation.y
                            self.snapTrans[2] = odom_base_1.transform.translation.z
                            self.snapRot = odom_base_1.transform.rotation
                            self.index = 1
                        case 1:
                            odom_trans.header.stamp = now.to_msg()
                            odom_trans.transform.translation.x = self.snapTrans[0]
                            odom_trans.transform.translation.y = self.snapTrans[1]
                            odom_trans.transform.translation.z = self.snapTrans[2]
                            odom_trans.transform.rotation.x = self.snapRot.x
                            odom_trans.transform.rotation.y = self.snapRot.y
                            odom_trans.transform.rotation.z = self.snapRot.z
                            odom_trans.transform.rotation.w = self.snapRot.w
                            self.get_logger().info(str(self.snapTrans)) 
                            self.broadcaster.sendTransform(odom_trans)
            case 1:
                try:
                    odom_base_1 = self.tf_buffer.lookup_transform('actual_odom','robot2/end-dummy',rclpy.time.Time())
                except TransformException as ex:
                    odom_trans.header.stamp = now.to_msg()
                    odom_trans.transform.translation.x = Trans[0]
                    odom_trans.transform.translation.y = Trans[1]
                    odom_trans.transform.translation.z = Trans[2]
                    odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 0) 
                    self.broadcaster.sendTransform(odom_trans)
                    return
                Trans[0] = odom_base_1.transform.translation.x
                Trans[1] = odom_base_1.transform.translation.y
                Trans[2] = odom_base_1.transform.translation.z
                Rot = odom_base_1.transform.rotation
                self.index = 0
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = Trans[0]
                odom_trans.transform.translation.y = Trans[1]
                odom_trans.transform.translation.z = Trans[2]
                odom_trans.transform.rotation.x = Rot.x
                odom_trans.transform.rotation.y = Rot.y
                odom_trans.transform.rotation.z = Rot.z
                odom_trans.transform.rotation.w = Rot.w
                self.broadcaster.sendTransform(odom_trans)

        
        





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