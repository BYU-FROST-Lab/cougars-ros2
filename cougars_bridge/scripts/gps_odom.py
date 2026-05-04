#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from message_filters import Subscriber, ApproximateTimeSynchronizer
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
        - gps_odom (nav_msgs/msg/Odometry)
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
        self.origin_sub = self.create_subscription(GeoPoint, 'origin', self.origin_callback, origin_qos)

        # Subscribe to NavSatFix
        self.fix_sub = Subscriber(self, NavSatFix, 'fix')

        # Subscribe to GPSFix to get covariance
        self.extended_fix_sub = Subscriber(self, GPSFix, 'extended_fix')

        # Message synchronizer enabling callback to use both fix and extended fix
        # This approch is necessary because currently covariance is only in the fix message, not extended fix
        self.ts = ApproximateTimeSynchronizer(
            [self.extended_fix_sub, self.fix_sub],
            queue_size=10,
            slop=0.001
        )
        self.ts.registerCallback(self.gps_callback)

        self.min_sats = 5  # Minimum number of satellites

        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, 'gps_odom', 10)

    def origin_callback(self, msg: GeoPoint):
        self.origin = msg
        self.get_logger().info(
            f"Received origin: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )

    def gps_callback(self, extended_msg: GPSFix, fix_msg: NavSatFix):
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

        # Access the altitude (z) value from the NavSatFix message
        z = extended_msg.altitude - self.origin.altitude

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
