#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
import time



class MastTest(Node):


    def __init__(self):
        super().__init__('mast_test')


        self.distance = 0

        self.vehicle_id = self.declare_parameter('vehicle_id', 'coug2').value
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/ttyAMA0').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)
        self.remote_device = None
        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
                    # Register XBee data receive callback
            self.device.add_data_received_callback(self.data_receive_callback)
            self.get_logger().info("RF Bridge node started using digi-xbee library.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")

    def data_receive_callback(self, data):
        payload = data.data.decode('utf-8', errors='replace')
        fields = payload.split(',')
        message_type = fields[0]
        
        

        if message_type == "START":
            test_name = fields[1]
            test_type = int(fields[2])
            distance = fields[3]
            if test_type == 0:
                self.run_normal_test(test_name, distance)
        elif message_type == "DISCOVERY":
            self.remote_device = data.remote_device.get_64bit_addr()
            self.get_logger().info(f"Found remote device {self.remote_device}")
            self.device.send_data_broadcast(f"DISCOVERY_ACK,{self.remote_device}")

    def run_normal_test(self, test_name, distance ):
        packets_sent = 0
        num_bytes = 128
        while packets_sent < 300:
            msg = f"{test_name},{packets_sent},{self.get_clock().now()},{num_bytes},{distance},{self.vehicle_id}"

            # make 128 bytes
            msg = msg.ljust(num_bytes, ' ')

            self.device.send_data(self.remote_device, msg)
            packets_sent += 1
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = MastTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
