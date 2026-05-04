#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geographic_msgs.msg import GeoPoint


class OriginPublisher(Node):
    """
    Publishes the ENU origin as a geographic_msgs/GeoPoint on the "origin" topic.

    Uses transient-local durability so any node that subscribes after this node
    starts will still receive the origin without needing to be running first.

    Parameters:
        origin.latitude  (float, degrees)
        origin.longitude (float, degrees)
        origin.altitude  (float, meters)
    """

    def __init__(self):
        super().__init__('origin_publisher')

        self.declare_parameter('use_param_origin', False)
        self.declare_parameter('origin.latitude', 40.2470633)
        self.declare_parameter('origin.longitude', -111.6446798)
        self.declare_parameter('origin.altitude', 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(GeoPoint, 'origin', qos)

        if self.get_parameter('use_param_origin').get_parameter_value().bool_value:
            self.publish_param_origin()
        else:
            self.get_logger().info("use_param_origin is false; not publishing origin.")

    def publish_param_origin(self):
        msg = GeoPoint()
        msg.latitude  = self.get_parameter('origin.latitude').get_parameter_value().double_value
        msg.longitude = self.get_parameter('origin.longitude').get_parameter_value().double_value
        msg.altitude  = self.get_parameter('origin.altitude').get_parameter_value().double_value

        self.pub.publish(msg)
        self.get_logger().info(
            f"Published origin: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OriginPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
