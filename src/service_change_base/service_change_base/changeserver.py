from escalador_interfaces.srv import ChangeBase                                                           # CHANGE

import rclpy
from rclpy.node import Node


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