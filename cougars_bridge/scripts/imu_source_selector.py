#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class ImuSourceSelector(Node):
    """
    Selects a configured IMU source topic and republishes it to a
    canonical IMU topic for the rest of the system.

    Example:
        setrac/imu/data  --->  imu/data
        /sim/imu/data     --->  imu/data
    """

    def __init__(self):
        super().__init__("imu_source_selector")

        # Parameters
        self.declare_parameter(
            "input_topic",
            "setrac/imu/data"
        )

        self.declare_parameter(
            "output_topic",
            "imu/data"
        )

        self.declare_parameter(
            "queue_size",
            10
        )

        self.input_topic = (
            self.get_parameter("input_topic")
            .get_parameter_value()
            .string_value
        )

        self.output_topic = (
            self.get_parameter("output_topic")
            .get_parameter_value()
            .string_value
        )

        self.queue_size = (
            self.get_parameter("queue_size")
            .get_parameter_value()
            .integer_value
        )

        # Prevent accidental loopback
        if self.input_topic == self.output_topic:
            self.get_logger().fatal(
                "input_topic and output_topic cannot be the same"
            )
            raise RuntimeError(
                "input_topic and output_topic cannot be the same"
            )

        # Publisher
        self.pub = self.create_publisher(
            Imu,
            self.output_topic,
            self.queue_size
        )

        # Subscriber
        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            self.queue_size
        )

        self.message_count = 0

        self.get_logger().info(
            f"Republishing IMU data:\n"
            f"  Input : {self.input_topic}\n"
            f"  Output: {self.output_topic}"
        )

    def imu_callback(self, msg: Imu):
        """
        Forward IMU message to canonical topic.
        """

        self.pub.publish(msg)

        self.message_count += 1

        # Optional periodic debug
        if self.message_count % 100 == 0:
            self.get_logger().debug(
                f"Forwarded {self.message_count} IMU messages"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ImuSourceSelector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()