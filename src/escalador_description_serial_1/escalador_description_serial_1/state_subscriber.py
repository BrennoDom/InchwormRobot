from math import sin, cos, pi
import rclpy
import copy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.logging import LoggingSeverity
from tf2_ros import TransformBroadcaster, TransformStamped

class StateSubscriber(Node):

    def __init__(self):
        
        super().__init__('state_subscriber')

        qos_profile = QoSProfile(depth=10)
        self.joint_sub = self.create_subscription(JointState, 'robot2/joint_states', self.listener_callback, qos_profile)
        self.nodeName = self.get_name()
        self.joint_sub
        self.flag = False

    def listener_callback(self, msg):
       self.position =[0,0,0,0,0,0]

       self.position[0] = msg.position[0]
       self.position[1] = msg.position[1]
       self.position[2] = msg.position[2]
       self.position[3] = msg.position[3]
       self.position[4] = msg.position[4]
       self.position[5] = msg.position[5]
 

    def get_angles(self):
        return copy.deepcopy(self.position)

  

def main(args=None):
    rclpy.init(args=args)
    state_subscriber = StateSubscriber()
    executor = SingleThreadedExecutor()   
    executor.add_node(state_subscriber)
    while rclpy.ok():
        executor.spin_once()
        rclpy.logging._root_logger.log(format(state_subscriber.get_angles()), LoggingSeverity.INFO)
    state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()