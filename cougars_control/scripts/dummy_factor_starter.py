#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from std_srvs.srv import SetBool
import time
class FactorStarter(Node):
    gotResponse=0
    def __init__(self):
        super().__init__('dummy_factor_graph_starter')
        self.client = self.create_client(SetBool, 'init_factor_graph')
        self.get_logger().info("dummy factor starter created")
        while self.gotResponse<10: #really stupid solution will fix later trust frfr
            self.get_logger().info("trying to start factor graph, "+f"gotRes = {self.gotResponse}")
            self.start_factor_graph()
            time.sleep(.75)
            self.gotResponse+=1
        self.get_logger().info("finished trying")
    def start_factor_graph(self):
        request = SetBool.Request()
        request.data=True
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.handle_factor_response)
    def handle_factor_response(self,future):
        self.get_logger().info(f"response called")
        try:
            response = future.result()
            if(response.success):
                self.gotResponse=10
            self.get_logger().info(f'Response: success={response.success}, message="{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = FactorStarter()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
