import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from pymavlink import mavutil


class MAVLinkBridge(Node):
    def __init__(self):
        super().__init__("mavlink_bridge")
        self.publisher = self.create_publisher(FluidPressure, "pressure/bar30", 10)
        self.mavlink_connection = mavutil.mavlink_connection(
            "udpin:192.168.2.103:15550"
        )
        self.get_logger().info("Waiting for MAVLink messages...")
        self.mavlink_connection.wait_heartbeat()
        self.timer = self.create_timer(0.1, self.publish_pressure)

    def publish_pressure(self):
        msg = self.mavlink_connection.recv_match(type="SCALED_PRESSURE", blocking=False)
        if msg:
            pressure_msg = FluidPressure()
            # TODO GET TIME FROM MESSAGE??
            # pressure_msg.header.stamp = self.
            pressure_msg.fluid_pressure = msg.press_abs * 100  # Conversion to Pa
            self.publisher.publish(pressure_msg)
            # self.get_logger().info("Published Pressure data")


def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
