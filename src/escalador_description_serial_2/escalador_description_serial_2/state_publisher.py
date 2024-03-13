from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    J1 = 0.
    J2 = 0.
    J3 = 0.
    J4 = 0.
    J5 = 0.
    J6 = 0.0

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.timer_ = self.create_timer(0.1, self.publish_joint)
        self.get_logger().info("{0} started".format(self.nodeName))

        self.create_subscription(JointState,'/joint_states',self.joint_callback,qos_profile)

        degree = pi / 180.0
        loop_rate = self.create_rate(30)



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
        #odom_trans = TransformStamped()
        #odom_trans.header.frame_id = 'robot2/LINK_7'
        #odom_trans.child_frame_id = 'robot1/dummy_base'
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['J1', 'J2', 'J3','J4', 'J5', 'J6']
        joint_state.position = [self.J1, -self.J2, -self.J3, self.J4, -self.J5, self.J6]

        # update transform
        # (moving in a circle with radius=2)
      #  odom_trans.header.stamp = now.to_msg()
      #  odom_trans.transform.translation.x = 0.0
      #  odom_trans.transform.translation.y = 0.0
      #  odom_trans.transform.translation.z = 0.0
      #  odom_trans.transform.rotation = \
      #  euler_to_quaternion(0.0, 0.0, 0.0) # roll,pitch,yaw

                # send the joint state and transform
        self.joint_pub.publish(joint_state)
      #  self.broadcaster.sendTransform(odom_trans)

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