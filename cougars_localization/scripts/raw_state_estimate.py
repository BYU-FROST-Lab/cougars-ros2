#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RawStateEstimate(Node):
    '''
    :author: Clayton Smith
    :date: May 2026

    Combines GPS x/y position from gps_odom with orientation from imu/data
    into a single Odometry message published as state_estimate.

    The latest IMU orientation
    is cached and stamped onto each GPS-triggered output message.

    Subscribes:
        - gps/odom (nav_msgs/msg/Odometry)
        - imu/data (sensor_msgs/msg/Imu)
    Publishes:
        - state_estimate (nav_msgs/msg/Odometry)
    '''

    def __init__(self):
        super().__init__('raw_state_estimate')

        self.latest_imu: Imu | None = None

        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.create_subscription(Odometry, 'gps/odom', self.gps_odom_callback, 10)

        self.publisher = self.create_publisher(Odometry, 'state_estimate', 10)

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def gps_odom_callback(self, msg: Odometry):
        if self.latest_imu is None:
            self.get_logger().warn(
                'No IMU data received yet, skipping state estimate', throttle_duration_sec=10
            )
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id

        out.pose.pose.position = msg.pose.pose.position
        out.pose.pose.orientation = self.latest_imu.orientation

        # Copy GPS position covariance (rows/cols 0-2) and IMU orientation covariance (rows/cols 3-5)
        # Odometry pose covariance is 6x6 row-major: [x, y, z, roll, pitch, yaw]
        gps_cov = msg.pose.covariance
        imu_cov = self.latest_imu.orientation_covariance  # 3x3 row-major

        out.pose.covariance[0] = gps_cov[0]   # xx
        out.pose.covariance[7] = gps_cov[7]   # yy
        out.pose.covariance[14] = gps_cov[14] # zz

        out.pose.covariance[21] = imu_cov[0]  # roll-roll
        out.pose.covariance[28] = imu_cov[4]  # pitch-pitch
        out.pose.covariance[35] = imu_cov[8]  # yaw-yaw

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RawStateEstimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()