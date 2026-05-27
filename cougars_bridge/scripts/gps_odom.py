#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

EARTH_RADIUS_METERS       = 6371000

class NavSatFixToOdom(Node):
    '''
    :author: Braden Meyers
    :date: September 2024

    A simple ROS2 node that subscribes to the extended_fix topic and converts the GPS data to Odometry messages.
    The GPS data is converted from latitude, longitude, and altitude to local Cartesian coordinates.

    Subscribes:
        - extended_fix (gps_msgs/msg/GPSFix)
        - origin (geographic_msgs/msg/GeoPoint)
    Publishes:
        - gps/odom (nav_msgs/msg/Odometry)
    '''
    def __init__(self):
        '''
        Creates a new NavSatFixToOdom node.
        '''
        super().__init__('gps_odom')

        self.origin = None

        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.origin_sub = self.create_subscription(GeoPoint, '/origin', self.origin_callback, origin_qos)

        # Subscribe to NavSatFix and GPSFix separately and cache the latest NavSatFix
        self.last_fix_msg = None
        self.last_fix_time_ns = None
        # maximum allowed age of cached covariance (seconds)
        self.fix_cov_max_age = 3.0

        # standard rclpy subscriptions (no message_filters)
        self.create_subscription(NavSatFix, 'fix', self.fix_callback, 10)
        self.create_subscription(GPSFix, 'extended_fix', self.extended_fix_callback, 10)

        self.min_sats = 5  # Minimum number of satellites

        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, 'gps/odom', 10)

    def origin_callback(self, msg: GeoPoint):
        self.origin = msg
        self.get_logger().info(
            f"Received origin: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )

    def fix_callback(self, msg: NavSatFix):
        # cache the latest NavSatFix (for covariance)
        self.last_fix_msg = msg
        self.last_fix_time_ns = self.get_clock().now().nanoseconds

    def extended_fix_callback(self, extended_msg: GPSFix):
        '''
        Callback function for the GPSFix subscription.
        Converts the GPS data to Odometry messages and publishes them.

        :param extended_msg: The GPSFix message received from the extended_fix topic.
        '''
        if self.origin is None:
            self.get_logger().warn("No origin received yet, skipping GPS reading", throttle_duration_sec=10)
            return

        # Filter out bad readings based on the number of satellites (if available)
        if extended_msg.status.satellites_used < self.min_sats or extended_msg.latitude < 0.1:
            self.get_logger().warn(f"Bad GPS status, skipping this GPS reading. Sat Used: {extended_msg.status.satellites_used}", throttle_duration_sec=10)
            return

        if math.isnan(extended_msg.latitude) or math.isnan(extended_msg.longitude) or math.isnan(extended_msg.altitude):
            self.get_logger().warn("NaN detected in GPS position, skipping this reading", throttle_duration_sec=10)
            return

        # Convert latitude/longitude to local Cartesian coordinates
        x, y = self.CalculateHaversine(
            self.origin.latitude,
            self.origin.longitude,
            extended_msg.latitude,
            extended_msg.longitude
        )

        # Access the altitude (z) value from the GPSFix message
        z = extended_msg.altitude - self.origin.altitude

        # Fill in the odometry message
        odom = Odometry()
        odom.header.stamp = extended_msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "gnss_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z  # Use the altitude as the z-value

        # Populate covariance from the most recent NavSatFix if available and recent
        if self.last_fix_msg is not None and self.last_fix_time_ns is not None:
            age_s = (self.get_clock().now().nanoseconds - self.last_fix_time_ns) / 1e9
            if age_s <= self.fix_cov_max_age:
                cov = self.last_fix_msg.position_covariance
                odom.pose.covariance[0] = cov[0]  # xx
                odom.pose.covariance[7] = cov[4]  # yy
                odom.pose.covariance[14] = cov[8]  # zz
            else:
                self.get_logger().warn(f"Cached fix too old ({age_s:.1f}s), publishing without covariance", throttle_duration_sec=10)
        else:
            self.get_logger().warn("No cached NavSatFix available, publishing without covariance", throttle_duration_sec=10)

        # Publish the odometry message
        self.publisher.publish(odom)

    def CalculateHaversine(self, refLat, refLong, pointLat, pointLong):
        # convert GPS coordinates to radians
        ref_lat_rad     = math.radians(refLat)
        ref_long_rad    = math.radians(refLong)
        point_lat_rad   = math.radians(pointLat)
        point_lon_rad   = math.radians(pointLong)

        # calculate distance and direction from reference point to GPS coordinate
        delta_lon = point_lon_rad - ref_long_rad
        delta_lat = point_lat_rad - ref_lat_rad
        a = math.sin(delta_lat/2)**2 + math.cos(ref_lat_rad) * math.cos(point_lat_rad) * math.sin(delta_lon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = EARTH_RADIUS_METERS * c
        theta = math.atan2(math.sin(delta_lon) * math.cos(point_lat_rad), math.cos(ref_lat_rad) * math.sin(point_lat_rad) - math.sin(ref_lat_rad) * math.cos(point_lat_rad) * math.cos(delta_lon))

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

if __name__ == '__main__':
    main()
