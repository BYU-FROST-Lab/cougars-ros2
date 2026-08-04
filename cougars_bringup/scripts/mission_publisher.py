#!/usr/bin/env python3

"""Loads a mission file and publishes it as a geographic_msgs/RouteNetwork message.

Two input formats:
  *.json  keyed by namespace, waypoints already in lat/lon
  *.csv   columns agent,ctrl_index,x,y -- ENU metres from the fleet origin, grouped
          by agent number and mapped to a namespace via the agent_namespaces param
"""

import collections
import csv
import json
import math
import struct
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geographic_msgs.msg import GeoPoint, RouteNetwork, WayPoint, KeyValue
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID


# Spherical earth, same radius gps_odom.py and waypoint_controller.cpp use. Do not
# swap in an ellipsoidal library here -- _enu_to_lla must be the exact inverse of
# haversine_to_enu in waypoint_controller.cpp or waypoints land ~0.3% off.
EARTH_RADIUS_METERS = 6371000


def _enu_to_lla(ref_lat: float, ref_lon: float, x: float, y: float) -> tuple:
    """ENU metres (x=East, y=North) from an origin -> WGS84 lat/lon in degrees."""
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    d = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(x, y)

    lat_rad = math.asin(math.sin(ref_lat_rad) * math.cos(d / EARTH_RADIUS_METERS) +
                        math.cos(ref_lat_rad) * math.sin(d / EARTH_RADIUS_METERS) * math.cos(theta))
    lon_rad = ref_lon_rad + math.atan2(
        math.sin(theta) * math.sin(d / EARTH_RADIUS_METERS) * math.cos(ref_lat_rad),
        math.cos(d / EARTH_RADIUS_METERS) - math.sin(ref_lat_rad) * math.sin(lat_rad))

    return math.degrees(lat_rad), math.degrees(lon_rad)


def _load_csv(path: str, agent_map: dict, ref_lat: float, ref_lon: float, logger=None) -> dict:
    """Read agent,ctrl_index,x,y and return {namespace: {defaults, waypoints}}."""
    grouped = collections.defaultdict(list)
    with open(path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            agent = row['agent'].strip()
            if agent not in agent_map:
                if logger:
                    logger.warning(f'CSV agent "{agent}" not in agent_namespaces; skipping.')
                continue
            grouped[agent_map[agent]].append(
                (int(row['ctrl_index']), float(row['x']), float(row['y']))
            )

    entries = {}
    for namespace, rows in grouped.items():
        rows.sort(key=lambda r: r[0])
        waypoints = []
        for _, x, y in rows:
            lat, lon = _enu_to_lla(ref_lat, ref_lon, x, y)
            waypoints.append({
                'lat': lat,
                'lon': lon,
                'z': 0.0,
                'depth_ref': 'surface',
                'park': False,
            })
        # ponytail: no per-mission defaults in the CSV; _build_route_network's own
        # fallbacks apply. Add a defaults block here if the planner ever emits speed.
        entries[namespace] = {'waypoints': waypoints}
    return entries


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


# Top-level keys that describe the mission file itself rather than a
# per-vehicle mission entry (see config/missions/*.yaml).
_METADATA_KEYS = {'mission_type', 'origin'}


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
        self.declare_parameter('agent_namespaces', ['1:coug2', '2:coug3'])

        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        mission_key = self.get_parameter('mission_key').get_parameter_value().string_value
        topic_override = self.get_parameter('topic').get_parameter_value().string_value

        if not mission_file:
            self.get_logger().error('Parameter "mission_file" is required.')
            sys.exit(1)

        if mission_file.lower().endswith('.csv'):
            agent_map = dict(
                pair.split(':', 1) for pair in
                self.get_parameter('agent_namespaces').get_parameter_value().string_array_value
            )
            origin = self._wait_for_origin()
            try:
                data = _load_csv(mission_file, agent_map,
                                 origin.latitude, origin.longitude, self.get_logger())
            except (OSError, KeyError, ValueError) as e:
                self.get_logger().error(f'Failed to load mission file "{mission_file}": {e}')
                sys.exit(1)
        else:
            try:
                with open(mission_file, 'r') as f:
                    data = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                self.get_logger().error(f'Failed to load mission file "{mission_file}": {e}')
                sys.exit(1)

        if not isinstance(data, dict) or not data:
            self.get_logger().error('Mission file must be a YAML mapping with at least one topic key.')
            sys.exit(1)

        # Determine which key(s) to publish, skipping mission-file metadata
        # keys (mission_type, origin) that aren't per-vehicle mission entries.
        if mission_key:
            if mission_key not in data or mission_key in _METADATA_KEYS:
                self.get_logger().error(f'mission_key "{mission_key}" not found in file.')
                sys.exit(1)
            entries = {mission_key: data[mission_key]}
        else:
            entries = {k: v for k, v in data.items() if k not in _METADATA_KEYS}

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pubs = {}
        self.messages = {}

        for key, value in entries.items():
            if isinstance(value, list):
                # Legacy format: array of waypoints
                defaults = {}
                waypoints = value
                entry_topic = None
            elif isinstance(value, dict):
                defaults = value.get('defaults', {})
                waypoints = value.get('waypoints', [])
                entry_topic = value.get('topic')
            else:
                self.get_logger().warning(f'Skipping key "{key}": unexpected format.')
                continue

            publish_topic = topic_override or entry_topic or key

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

    def _wait_for_origin(self) -> GeoPoint:
        """Block until origin_publisher's /origin arrives. It is transient-local, so a
        late subscriber still gets the latched value and this returns immediately."""
        self._origin = None
        sub = self.create_subscription(
            GeoPoint, '/origin', lambda msg: setattr(self, '_origin', msg),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.get_logger().info('Waiting for /origin before converting CSV waypoints...')
        while self._origin is None:
            rclpy.spin_once(self, timeout_sec=1.0)
            if self._origin is None:
                self.get_logger().warning(
                    'Still waiting for /origin (is origin_publisher running with '
                    'use_param_origin: true?)')
        self.destroy_subscription(sub)
        self.get_logger().info(
            f'Received origin: lat={self._origin.latitude}, lon={self._origin.longitude}')
        return self._origin

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