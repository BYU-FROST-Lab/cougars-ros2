#!/usr/bin/env python3

"""Loads a mission JSON file and publishes it as a geographic_msgs/RouteNetwork message."""

import json
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geographic_msgs.msg import GeoPoint, RouteNetwork, WayPoint, KeyValue
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID


def _make_uuid(index: int) -> UUID:
    """Encode an integer into bytes 12-15 of a UUID (big-endian), rest zero."""
    uuid = UUID()
    uuid.uuid = [0] * 16
    packed = struct.pack('>I', index & 0xFFFFFFFF)
    uuid.uuid[12] = packed[0]
    uuid.uuid[13] = packed[1]
    uuid.uuid[14] = packed[2]
    uuid.uuid[15] = packed[3]
    return uuid


def _kv(key: str, value: str) -> KeyValue:
    kv = KeyValue()
    kv.key = key
    kv.value = value
    return kv


def _build_route_network(defaults: dict, waypoints: list, topic: str, node: Node) -> RouteNetwork:
    msg = RouteNetwork()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'wgs84'

    mission_id = int(defaults.get('mission_id', 0))
    msg.id = _make_uuid(mission_id)

    msg.props.append(_kv('speed', str(defaults.get('speed', 50.0))))
    msg.props.append(_kv('slip_radius', str(defaults.get('slip_radius', 2.0))))
    msg.props.append(_kv('capture_radius', str(defaults.get('capture_radius', 10.0))))

    for i, wp_data in enumerate(waypoints):
        wp = WayPoint()
        wp.id = _make_uuid(i)

        wp.position = GeoPoint()
        wp.position.latitude = float(wp_data.get('lat', 0.0))
        wp.position.longitude = float(wp_data.get('lon', 0.0))
        wp.position.altitude = float(wp_data.get('z', 0.0))

        wp.props.append(_kv('depth_ref', wp_data.get('depth_ref', 'surface')))
        wp.props.append(_kv('park', 'true' if wp_data.get('park', False) else 'false'))

        if 'speed' in wp_data:
            wp.props.append(_kv('speed', str(wp_data['speed'])))
        if 'slip_radius' in wp_data:
            wp.props.append(_kv('slip_radius', str(wp_data['slip_radius'])))
        if 'capture_radius' in wp_data:
            wp.props.append(_kv('capture_radius', str(wp_data['capture_radius'])))

        msg.points.append(wp)

    return msg


class MissionPublisher(Node):
    def __init__(self):
        super().__init__('mission_publisher')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('mission_key', '')
        self.declare_parameter('topic', 'mission')

        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        mission_key = self.get_parameter('mission_key').get_parameter_value().string_value
        topic_override = self.get_parameter('topic').get_parameter_value().string_value

        if not mission_file:
            self.get_logger().error('Parameter "mission_file" is required.')
            sys.exit(1)

        try:
            with open(mission_file, 'r') as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            self.get_logger().error(f'Failed to load mission file "{mission_file}": {e}')
            sys.exit(1)

        if not isinstance(data, dict) or not data:
            self.get_logger().error('Mission file must be a JSON object with at least one topic key.')
            sys.exit(1)

        # Determine which key(s) to publish
        if mission_key:
            if mission_key not in data:
                self.get_logger().error(f'mission_key "{mission_key}" not found in file.')
                sys.exit(1)
            entries = {mission_key: data[mission_key]}
        else:
            entries = data

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pubs = {}
        self.messages = {}

        for key, value in entries.items():
            publish_topic = topic_override if topic_override else key

            if isinstance(value, list):
                # Legacy format: array of waypoints
                defaults = {}
                waypoints = value
            elif isinstance(value, dict):
                defaults = value.get('defaults', {})
                waypoints = value.get('waypoints', [])
            else:
                self.get_logger().warning(f'Skipping key "{key}": unexpected format.')
                continue

            msg = _build_route_network(defaults, waypoints, publish_topic, self)
            self.pubs[publish_topic] = self.create_publisher(
                RouteNetwork, publish_topic, latched_qos
            )
            self.messages[publish_topic] = msg
            self.get_logger().info(
                f'Loaded {len(waypoints)} waypoint(s) for topic "{publish_topic}" '
                f'(mission_id={defaults.get("mission_id", 0)}).'
            )

        if not self.pubs:
            self.get_logger().error('No valid missions to publish.')
            sys.exit(1)

        # Publish once after a short delay so the publisher is registered
        self._timer = self.create_timer(0.1, self._publish_once)

    def _publish_once(self):
        self._timer.cancel()
        for topic, pub in self.pubs.items():
            pub.publish(self.messages[topic])
            self.get_logger().info(f'Published RouteNetwork on "{topic}".')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
