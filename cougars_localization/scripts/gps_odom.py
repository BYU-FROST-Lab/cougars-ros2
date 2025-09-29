#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from cougars_interfaces.msg import SystemControl
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import yaml

EARTH_RADIUS_METERS = 6371000


class NavSatFixToOdom(Node):
    """
    :author: Braden Meyers
    :date: September 2024

    A simple ROS2 node that subscribes to the extended_fix topic and converts the GPS data to Odometry messages.
    The GPS data is converted from latitude, longitude, and altitude to local Cartesian coordinates.

    Subscribes:
        - extended_fix (gps_msgs/msg/GPSFix)
    Publishes:
        - gps_odom (nav_msgs/msg/Odometry)
    """

    def __init__(self):
        """
        Creates a new NavSatFixToOdom node.
        """
        super().__init__("gps_odom")

        # Declare parameters for the origin (datum)
        self.declare_parameter("origin.latitude", 40.2470633)  # 34.0219
        """
        :param origin.latitude: The latitude of the origin (datum) for the local Cartesian projection. The default value is 34.0219.
        """

        self.declare_parameter("origin.longitude", -111.6446798)  # -118.4814
        """
        :param origin.longitude: The longitude of the origin (datum) for the local Cartesian projection. The default value is -118.4814.
        """

        self.declare_parameter("origin.altitude", 0.0)
        """
        :param origin.altitude: The altitude of the origin (datum) for the local Cartesian projection. The default value is 0.0.
        """
        self.declare_parameter("mission_file_path", "")

        # Subscribe to NavSatFix
        self.fix_sub = Subscriber(self, NavSatFix, "fix")
        """
        Subscription to the "extended_fix" topic with the message type GPSFix.
        """

        # Subscribe to GPSFix to get covariance
        self.extended_fix_sub = Subscriber(self, GPSFix, "extended_fix")

        # Message synchronizer enabling callback to use both fix and extended fix
        # This approch is necessary because currently covariance is only in the fix message, not extended fix
        self.ts = ApproximateTimeSynchronizer(
            [self.extended_fix_sub, self.fix_sub], queue_size=10, slop=0.001
        )
        self.ts.registerCallback(self.gps_callback)

        self.min_sats = 5  # Minimum number of satellites

        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, "gps_odom", 10)
        """
        Publisher for the "gps_odom" topic with the message type Odometry.
        """
        # Set the origin from the mission file path
        self.set_origin()

        # Subscriber for setting the origin
        # Create the system services subscriber and publisher

        self.system_status_sub = self.create_subscription(
            SystemControl, "system/status", self.system_status_callback, 1
        )

    def set_origin(self):
        """
        Load the origin (latitude, longitude, altitude) from the YAML file
        and set them as parameters: origin.latitude, origin.longitude, origin.altitude
        """
        yaml_path = self.get_parameter("mission_file_path").value
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
                origin = data.get("origin_lla", {})
                lat = origin.get("latitude")
                lon = origin.get("longitude")
                alt = origin.get("altitude")

                if lat is not None and lon is not None and alt is not None:
                    self.set_parameters(
                        [
                            rclpy.parameter.Parameter(
                                "origin.latitude", rclpy.Parameter.Type.DOUBLE, lat
                            ),
                            rclpy.parameter.Parameter(
                                "origin.longitude", rclpy.Parameter.Type.DOUBLE, lon
                            ),
                            rclpy.parameter.Parameter(
                                "origin.altitude", rclpy.Parameter.Type.DOUBLE, alt
                            ),
                        ]
                    )
                    self.get_logger().info(
                        f"Set origin to lat: {lat}, lon: {lon}, alt: {alt}"
                    )
                else:
                    self.get_logger().error(
                        "Origin fields missing in YAML. Parameters not updated."
                    )

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML for origin: {e}")

    def system_status_callback(self, msg):
        """
        Callback function for the init mission to set the origin.
        """
        init_bool = msg.start.data
        if init_bool:
            self.set_origin()
        else:
            self.get_logger().info(
                "GPS Odom node will not set new origin - Init flag false"
            )

    def gps_callback(self, extended_msg: GPSFix, fix_msg: NavSatFix):
        """
        Callback function for the GPSFix subscription.
        Converts the GPS data to Odometry messages and publishes them.

        :param extended_msg: The GPSFix message received from the extended_fix topic.
        """

        # Filter out bad readings based on the number of satellites (if available)
        if (
            extended_msg.status.satellites_used < self.min_sats
            or extended_msg.latitude < 0.1
        ):
            self.get_logger().warn(
                f"Bad GPS status, skipping this GPS reading. Sat Used: {extended_msg.status.satellites_used}",
                throttle_duration_sec=10,
            )
            return

        if (
            math.isnan(extended_msg.latitude)
            or math.isnan(extended_msg.longitude)
            or math.isnan(extended_msg.altitude)
        ):
            self.get_logger().warn(
                "NaN detected in GPS position, skipping this reading",
                throttle_duration_sec=10,
            )
            return

        # Convert latitude/longitude to local Cartesian coordinates
        x, y = self.CalculateHaversine(
            self.get_parameter("origin.latitude").get_parameter_value().double_value,
            self.get_parameter("origin.longitude").get_parameter_value().double_value,
            extended_msg.latitude,
            extended_msg.longitude,
        )

        # Access the altitude (z) value from the NavSatFix message
        z = (
            extended_msg.altitude
            - self.get_parameter("origin.altitude").get_parameter_value().double_value
        )

        # Fill in the odometry message
        odom = Odometry()
        odom.header.stamp = extended_msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "gnss_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z  # Use the altitude as the z-value

        # Set the covariance values for x, y, and z
        odom.pose.covariance[0] = fix_msg.position_covariance[0]  # xx
        odom.pose.covariance[7] = fix_msg.position_covariance[4]  # yy
        odom.pose.covariance[14] = fix_msg.position_covariance[8]  # zz

        # Publish the odometry message
        self.publisher.publish(odom)

    def CalculateHaversine(self, refLat, refLong, pointLat, pointLong):
        # convert GPS coordinates to radians
        ref_lat_rad = math.radians(refLat)
        ref_long_rad = math.radians(refLong)
        point_lat_rad = math.radians(pointLat)
        point_lon_rad = math.radians(pointLong)

        # calculate distance and direction from reference point to GPS coordinate
        delta_lon = point_lon_rad - ref_long_rad
        delta_lat = point_lat_rad - ref_lat_rad
        a = (
            math.sin(delta_lat / 2) ** 2
            + math.cos(ref_lat_rad)
            * math.cos(point_lat_rad)
            * math.sin(delta_lon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = EARTH_RADIUS_METERS * c
        theta = math.atan2(
            math.sin(delta_lon) * math.cos(point_lat_rad),
            math.cos(ref_lat_rad) * math.sin(point_lat_rad)
            - math.sin(ref_lat_rad) * math.cos(point_lat_rad) * math.cos(delta_lon),
        )

        # convert distance and direction to xy coordinates in meters
        y = d * math.cos(theta)
        x = d * math.sin(theta)
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
