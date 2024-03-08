from escalador_interfaces.srv import ChangeBase                                                           # CHANGE
import sys
import rclpy
from rclpy.node import Node


class ChangeBaseClient(Node):

    def __init__(self):
        super().__init__('chbase_client_async')
        self.cli = self.create_client(ChangeBase, 'SrvChangeBase')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChangeBase.Request()                                   # CHANGE

    def send_request(self):
        self.req.change = sys.argv[1] == 'True'
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    chbase_client = ChangeBaseClient()
    chbase_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(chbase_client)
        if chbase_client.future.done():
            try:
                response = chbase_client.future.result()
            except Exception as e:
                chbase_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                chbase_client.get_logger().info(
                    'Value %s for Base %s' %                                # CHANGE
                    (chbase_client.req.change, response.basename))  # CHANGE
            break

    chbase_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()