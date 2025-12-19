#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header, Bool
from cougars_interfaces.msg import SystemControl  # Change to your actual package name

import time


class SystemStatusPublisher(Node):
    def __init__(self):
        super().__init__('system_status_publisher')


        self.coug0_publisher_ = self.create_publisher(SystemControl, '/coug0/system/status', 1)
        self.coug1_publisher_ = self.create_publisher(SystemControl, '/coug1/system/status', 1)
        self.coug2_publisher_ = self.create_publisher(SystemControl, '/coug2/system/status', 1)
        self.coug3_publisher_ = self.create_publisher(SystemControl, '/coug3/system/status', 1)
        self.get_logger().info("SystemStatusPublisher node started. Preparing message...")

        self.publish_user_input()

    def publish_user_input(self):
        msg = SystemControl()

        # Fill header with timestamp
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'system_status_input'


        # TODO let the user put in the parameters in the command line
        # Prompt user for input
        try:
            start_input = input("Start the node? (y/n): ").strip().lower()
            rosbag_flag_input = input("Record rosbag? (y/n): ").strip().lower()
            rosbag_prefix_input = input("Enter rosbag prefix (string): ").strip()
            arm_thruster_input = input("Arm Thruster? (y/n): ").strip().lower()
            dvl_acoustics = input("Start DVL (y/n): ").strip().lower()

            msg.start = Bool(data=start_input == "y")
            msg.rosbag_flag = Bool(data=rosbag_flag_input == "y")
            msg.rosbag_prefix = rosbag_prefix_input
            msg.thruster_arm = Bool(data=arm_thruster_input == "y")
            msg.dvl_acoustics = Bool(data=dvl_acoustics == "y")

            # Publish message
            # TODO control which vehicles to publish to
            self.coug0_publisher_.publish(msg)
            self.coug1_publisher_.publish(msg)
            self.coug2_publisher_.publish(msg)
            self.coug3_publisher_.publish(msg)
            self.get_logger().info("Published SystemControl message.")
            self.get_logger().info(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}")

        except Exception as e:
            self.get_logger().error(f"Error getting user input: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusPublisher()

    # Allow time for reliable delivery
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
