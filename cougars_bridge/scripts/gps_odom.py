#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

EARTH_RADIUS_METERS       = 6371000

class NavSatFixToOdom(Node):
    '''
    :author: Braden Meyers
    :date: September 2024

    A simple ROS2 node that subscribes to the fix topic and converts the GPS data to Odometry messages.
    The GPS data is converted from latitude, longitude, and altitude to local Cartesian coordinates.

    The GPS source is chosen by the vehicle_fix_source relay, which republishes onto fix
    (e.g. imu/nav_sat_fix from the SBG driver, or gpsd's fix).

    The orientation is taken from the latest /imu/data message and copied into the
    published Odometry pose.

    Subscribes:
        - fix (sensor_msgs/msg/NavSatFix)
        - imu/data (sensor_msgs/msg/Imu)
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

        # Cache the latest IMU orientation
        self.last_imu_msg = None
        self.last_imu_time_ns = None
        # maximum allowed age of cached orientation (seconds)
        self.imu_max_age = 3.0

        # standard rclpy subscriptions (no message_filters)
        self.create_subscription(NavSatFix, 'fix', self.fix_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, 'gps/odom', 10)

    def origin_callback(self, msg: GeoPoint):
        self.origin = msg
        self.get_logger().info(
            f"Received origin: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )

    def imu_callback(self, msg: Imu):
        # cache the latest IMU message (for orientation)
        self.last_imu_msg = msg
        self.last_imu_time_ns = self.get_clock().now().nanoseconds

    def apply_orientation(self, odom: Odometry):
        '''
        Copies the most recent IMU orientation into the given Odometry message,
        if a recent IMU reading is available.
        '''
        if self.last_imu_msg is not None and self.last_imu_time_ns is not None:
            age_s = (self.get_clock().now().nanoseconds - self.last_imu_time_ns) / 1e9
            if age_s <= self.imu_max_age:
                odom.pose.pose.orientation = self.last_imu_msg.orientation
                for i in range(3):
                    for j in range(3):
                        odom.pose.covariance[21 + i * 6 + j] = self.last_imu_msg.orientation_covariance[i * 3 + j]
                return
            self.get_logger().warn(f"Cached IMU too old ({age_s:.1f}s), publishing without orientation", throttle_duration_sec=10)
        else:
            self.get_logger().warn("No cached IMU available, publishing without orientation", throttle_duration_sec=10)

    def fix_callback(self, msg: NavSatFix):
        '''
        Callback function for the NavSatFix subscription.
        Converts the GPS data to Odometry messages and publishes them.

        :param msg: The NavSatFix message received from the fix topic.
        '''
        if self.origin is None:
            self.get_logger().warn("No origin received yet, skipping GPS reading", throttle_duration_sec=10)
            return

        # Filter out readings without a position solution
        if msg.status.status < NavSatStatus.STATUS_FIX or abs(msg.latitude) < 0.1:
            self.get_logger().warn(f"No GPS fix, skipping this GPS reading. Status: {msg.status.status}", throttle_duration_sec=10)
            return

        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            self.get_logger().warn("NaN detected in GPS position, skipping this reading", throttle_duration_sec=10)
            return

        # Convert latitude/longitude to local Cartesian coordinates
        x, y = self.CalculateHaversine(
            self.origin.latitude,
            self.origin.longitude,
            msg.latitude,
            msg.longitude
        )

        # Access the altitude (z) value from the NavSatFix message
        z = msg.altitude - self.origin.altitude

        # Fill in the odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "gnss_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z  # Use the altitude as the z-value

        # NavSatFix covariance is ENU, which matches the map frame
        cov = msg.position_covariance
        odom.pose.covariance[0] = cov[0]  # xx
        odom.pose.covariance[7] = cov[4]  # yy
        odom.pose.covariance[14] = cov[8]  # zz

        # Populate orientation from the most recent IMU message
        self.apply_orientation(odom)

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
