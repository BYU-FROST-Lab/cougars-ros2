#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cougars_interfaces.msg import VehicleSetpoint
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

"""
Setpoint Transformer

Sits between guidance nodes and the controller. Converts ALTITUDE-mode setpoints
(depth field = desired meters above seafloor) into DEPTH-mode setpoints
(depth field = meters below surface) using live DVL altitude and depth sensor data.

Subscribes:
  - guidance/setpoint_raw (VehicleSetpoint)  — from mission/teleop nodes
  - depth/odom            (nav_msgs/Odometry) — actual vehicle depth
  - dvl/data              (dvl_msgs/DVL)       — altitude above seafloor

Publishes:
  - control/setpoint (VehicleSetpoint, mode always = DEPTH)
"""


class SetpointTransformer(Node):

    def __init__(self):
        super().__init__('setpoint_transformer')

        self.actual_depth = 0.0
        self.dvl_altitude = -1.0  # negative = no valid bottom lock

        dvl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.setpoint_pub = self.create_publisher(
            VehicleSetpoint, 'control/setpoint', qos_profile_system_default)

        self.create_subscription(
            VehicleSetpoint, 'guidance/setpoint_raw',
            self.setpoint_callback, qos_profile_system_default)

        self.create_subscription(
            Odometry, 'depth/odom',
            self.depth_callback, 10)

        self.create_subscription(
            DVL, 'dvl/data',
            self.dvl_callback, dvl_qos)

        self.get_logger().info('Setpoint Transformer started')

    def depth_callback(self, msg):
        # Negate ENU z to get positive-down depth
        self.actual_depth = -msg.pose.pose.position.z

    def dvl_callback(self, msg):
        self.dvl_altitude = msg.altitude  # meters above seafloor; negative if no bottom lock

    def setpoint_callback(self, msg):
        if msg.mode == VehicleSetpoint.DEPTH:
            self.setpoint_pub.publish(msg)
            return

        if msg.mode == VehicleSetpoint.ALTITUDE:
            if self.dvl_altitude <= 0.0:
                self.get_logger().warn(
                    'ALTITUDE setpoint received but DVL has no valid bottom lock — dropping',
                    throttle_duration_sec=2.0)
                return

            out = VehicleSetpoint()
            out.header = msg.header
            out.mode = VehicleSetpoint.DEPTH

            seafloor_depth = self.actual_depth + self.dvl_altitude
            out.depth = seafloor_depth - msg.depth  # msg.depth = desired altitude above bottom
            out.depth_valid = msg.depth_valid

            out.heading = msg.heading
            out.heading_valid = msg.heading_valid
            out.speed = msg.speed
            out.speed_valid = msg.speed_valid

            self.setpoint_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
