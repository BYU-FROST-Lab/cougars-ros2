#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from dvl_msgs.msg import DVL
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from cougars_interfaces.msg import MissionFeedback, SystemControl, UCommand, WaypointFeedback
from geographic_msgs.msg import GeoPoint, RouteNetwork

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import TransmitException
from pathlib import Path

import radio_protocol as rp

import json
import math
import traceback
import base64
import os
import subprocess
import time
import shutil

class RFBridge(Node):
    MAX_XBEE_PAYLOAD_BYTES = 90
    FRAGMENT_DATA_BYTES = 18

    def __init__(self):
        super().__init__('rf_bridge')

        # Debug mode
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        if self.debug_mode:
            self.get_logger().info("Debug mode enabled: Will log detailed packet information")

        # QoS profiles
        self.odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.dvl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # Data storage
        self.latest_state_estimate = "NO_DATA"
        self.latest_dvl_velocity = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_pressure = "NO_DATA"
        self.latest_mission_feedback = "NO_DATA"
        self.latest_waypoint_feedback = "NO_DATA"

        self.thruster_enable = False

        self.vehicle_id = self.declare_parameter('vehicle_ID', 0).value
        self.base_station_id = self.declare_parameter('base_station_id', 15).value

        # XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/ttyAMA0').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)
        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device on {self.xbee_port} at {self.xbee_baud} because of exception {e} ")
            return

        # ROS publishers and subscribers
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        self.init_publisher = self.create_publisher(SystemControl, 'system/control', 10)
        origin_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_publisher = self.create_publisher(GeoPoint, '/origin', origin_qos)
        self.mission_publisher = self.create_publisher(
            RouteNetwork,
            'mission',
            origin_qos,
        )

        self.e_kill_client = self.create_client(SetBool, "arm_thruster")

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)

        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery/data',
            self.battery_callback,
            10)
        
        self.state_estimate_sub = self.create_subscription(
            Odometry,
            'odometry/global',
            self.state_estimate_callback,
            10)

        self.dvl_sub = self.create_subscription(
            DVL,
            'dvl',
            self.dvl_callback,
            self.dvl_qos)
        
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            'pressure/data',
            self.pressure_callback,
            10)
        
        self.mission_feedback_sub = self.create_subscription(
            MissionFeedback,
            'mission_feedback',
            self.mission_feedback_callback,
            10)

        self.waypoint_feedback_sub = self.create_subscription(
            WaypointFeedback,
            'waypoint_feedback',
            self.waypoint_feedback_callback,
            10)
        self.ucommand_pub = self.create_publisher(UCommand, 'controls/command', 10)

        # Direct ROS topic for hardware control delivered over WiFi (bridged from
        # coug{id}/hardware_control by base_station_wifi.py's VehicleWifiConnection).
        self.hardware_control_sub = self.create_subscription(
            String,
            'hardware_control_cmd',
            self.hardware_control_topic_callback,
            10)

        # Register XBee data receive callback
        self.device.add_data_received_callback(self.data_receive_callback)
        self.get_logger().info("RF Bridge node started using digi-xbee library.")

        # Thread-safe shutdown flag
        self.running = True
        
        # File transfer state tracking
        self.active_transfers = {}  # transfer_id -> transfer_info
        self.file_chunks = {}       # transfer_id -> {chunk_num -> data}
        self.fragment_transfers = {}
        self.mission_transfers = {}  # transfer_id -> {"chunks": {}, "total": int, "address": addr}
        self.received_files_dir = "/tmp/received_missions"  # Directory to save received files
        os.makedirs(self.received_files_dir, exist_ok=True)

        self.VEHICLE_PARAMS_FILE = f"coug{self.vehicle_id}params.yaml"
        self.FLEET_PARAMS_FILE = "fleet_params.yaml"
        self.MISSION_FILE = "mission.yaml"

        self.HARDWARE_CONTROL_SCRIPTS = {
            int(rp.HardwareDevice.RELAY): os.path.expanduser("~/scripts/set_relay.sh"),
            int(rp.HardwareDevice.STROBE): os.path.expanduser("~/scripts/set_strobe.sh"),
        }
        self.HARDWARE_CONTROL_MODES = {
            int(rp.HardwareMode.AUTO): "auto",
            int(rp.HardwareMode.ON): "on",
            int(rp.HardwareMode.OFF): "off",
        }

    def battery_callback(self, msg):
        self.latest_battery = msg
        self.get_logger().debug("Updated battery data")

    def state_estimate_callback(self, msg):
        self.latest_state_estimate = msg
        self.get_logger().debug("Updated state estimate data")

    def dvl_callback(self, msg):
        self.latest_dvl_velocity = msg
        self.get_logger().debug("Updated DVL velocity data")

    def mission_feedback_callback(self, msg):
        self.latest_mission_feedback = msg
        self.get_logger().debug("Updated mission feedback data")

    def waypoint_feedback_callback(self, msg):
        self.latest_waypoint_feedback = msg
        self.get_logger().debug("Updated waypoint feedback data")

    def pressure_callback(self, msg):
        self.latest_pressure = msg
        self.get_logger().debug("Updated pressure data")

    def tx_callback(self, msg):
        try:
            message = msg.data
            self.device.send_data_broadcast(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            # self.get_logger().error(f"XBee transmission error: {str(e)}")
            # self.get_logger().error(traceback.format_exc())
            pass 

    def _send_raw_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().debug(f"Sent via XBee: {msg}")
            return True
        except TransmitException as e:
            self.get_logger().debug(f"XBee transmission error: {e}")
            return False
        except Exception as e:
            self.get_logger().debug(f"XBee transmission error: {e}")
            return False

    def send_message(self, msg, address):
        payload = msg.encode('utf-8') if isinstance(msg, str) else bytes(msg)
        if len(payload) <= self.MAX_XBEE_PAYLOAD_BYTES:
            return self._send_raw_message(payload, address)

        transfer_id = f"{time.time_ns() & 0xffffffffffff:x}"
        chunks = [
            payload[offset:offset + self.FRAGMENT_DATA_BYTES]
            for offset in range(0, len(payload), self.FRAGMENT_DATA_BYTES)
        ]
        for index, chunk in enumerate(chunks):
            fragment = json.dumps({
                "message": "FRAGMENT",
                "id": transfer_id,
                "i": index,
                "n": len(chunks),
                "data": base64.b64encode(chunk).decode('ascii'),
            }, separators=(',', ':'))
            if not self._send_raw_message(fragment, address):
                return False
            time.sleep(0.03)

        self.get_logger().debug(
            f"Sent fragmented message {transfer_id} in {len(chunks)} chunks"
        )
        return True

    def get_all_status_data(self):
        status_response = rp.StatusResponseMessage(src_id=self.vehicle_id)

        if self.latest_state_estimate != "NO_DATA":
            pose = self.latest_state_estimate.pose.pose
            status_response.x = pose.position.x
            status_response.y = pose.position.y
            status_response.depth = pose.position.z
            status_response.orientation_x = pose.orientation.x
            status_response.orientation_y = pose.orientation.y
            status_response.orientation_z = pose.orientation.z
            status_response.orientation_w = pose.orientation.w
            covariance = self.latest_state_estimate.pose.covariance
            status_response.cov_x = covariance[0]
            status_response.cov_y = covariance[7]
            status_response.cov_z = covariance[14]
            status_response.cov_roll = covariance[21]
            status_response.cov_pitch = covariance[28]
            status_response.cov_yaw = covariance[35]
        if self.latest_pressure != "NO_DATA":
            status_response.pressure = self.latest_pressure.fluid_pressure
            status_response.pressure_variance = self.latest_pressure.variance
        if self.latest_battery != "NO_DATA":
            status_response.battery_voltage = self.latest_battery.voltage
            status_response.battery_current = self.latest_battery.current
        if self.latest_dvl_velocity != "NO_DATA":
            status_response.dvl_velocity_x = self.latest_dvl_velocity.velocity.x
            status_response.dvl_velocity_y = self.latest_dvl_velocity.velocity.y
            status_response.dvl_velocity_z = self.latest_dvl_velocity.velocity.z
            status_response.dvl_altitude = self.latest_dvl_velocity.altitude
        if self.latest_waypoint_feedback != "NO_DATA":
            status_response.waypoint_state = self.latest_waypoint_feedback.state
            status_response.horizontal_distance_error = self.latest_waypoint_feedback.horizontal_distance_error
            status_response.depth_error = self.latest_waypoint_feedback.depth_error
            status_response.bearing_error = self.latest_waypoint_feedback.bearing_error
        if self.latest_mission_feedback != "NO_DATA":
            status_response.mission_id = self.latest_mission_feedback.mission_id
            status_response.mission_state = self.latest_mission_feedback.state
            status_response.waypoints_completed = self.latest_mission_feedback.waypoints_completed
            status_response.waypoints_total = self.latest_mission_feedback.waypoints_total
            status_response.elapsed_time = self.latest_mission_feedback.elapsed_time

        return status_response.pack()

    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data
            sender_id = payload[1] if len(payload) > 1 else None
            return_address = xbee_message.remote_device.get_64bit_addr()

            msg_id = payload[0] if len(payload) > 0 else None
            if payload == b"MISSION_DONE":
                return
            if msg_id is not None:
                self.get_logger().debug(f"Received message ID {msg_id} from {sender_id}")
            else:
                self.get_logger().debug(f"Received message with no ID from {sender_id}")

            # Check for JSON mission fragments first
            if isinstance(payload, (bytes, bytearray)) and payload.startswith(b"{\""):
                try:
                    decoded = json.loads(payload.decode("utf-8"))
                except Exception:
                    decoded = None
                if isinstance(decoded, dict) and decoded.get("message") == "MISSION_FRAGMENT":
                    self.handle_mission_fragment(decoded, return_address)
                    return
            
            if msg_id == int(rp.MessageID.REQUEST_STATUS):
                response = self.get_all_status_data()
                self.send_message(response, return_address)
                self.get_logger().debug(f"Received STATUS, responding with sensor data")
                self.get_logger().debug(f"Status response: {response}")
            elif msg_id == int(rp.MessageID.PING):
                response = rp.PingMessage(src_id=self.vehicle_id).pack()
                self.send_message(response, return_address)
                self.get_logger().debug(f"Received PING, responding with PING")
            elif msg_id == int(rp.MessageID.DISARM_THRUSTER):
                self.get_logger().info(f"Received E_KILL message")
                self.kill_thruster(return_address)
            elif msg_id == int(rp.MessageID.SYSTEM_CONTROL):
                self.get_logger().info(f"Received INIT command ")
                self.init_vehicle(payload, return_address)
            elif msg_id == int(rp.MessageID.ORIGIN_UPDATE):
                self.publish_origin(payload, return_address)
            elif msg_id == int(rp.MessageID.HARDWARE_CONTROL):
                self.get_logger().info("Received HARDWARE_CONTROL command")
                self.set_hardware_control(payload, return_address)
            elif msg_id == int(rp.MessageID.KEY_CONTROL):
                self.handle_key_control(payload)
            elif msg_id not in [int(rp.MessageID.PING), int(rp.MessageID.STATUS_RESPONSE), int(rp.MessageID.CONFIRM_DISARM_THRUSTER), int(rp.MessageID.CONFIRM_SYSTEM_CONTROL)]:
                # Try to publish unrecognized message types as missions
                self.publish_mission(payload)
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")
            # self.get_logger().error(traceback.format_exc())

    def handle_mission_fragment(self, fragment, return_address):
        transfer_id = fragment.get("id")
        chunk_index = fragment.get("i")
        total_chunks = fragment.get("n")
        data = fragment.get("data")

        if transfer_id is None or chunk_index is None or total_chunks is None or data is None:
            self.get_logger().warn(f"[MISSION_FRAGMENT] Invalid fragment structure: missing required fields")
            return

        transfer = self.mission_transfers.setdefault(transfer_id, {
            "chunks": {},
            "total": total_chunks,
            "address": return_address,
        })
        
        decoded_chunk = base64.b64decode(data)
        transfer["chunks"][chunk_index] = decoded_chunk
        
        self.get_logger().info(
            f"[MISSION_FRAGMENT] Received fragment {chunk_index + 1}/{total_chunks} "
            f"(transfer_id={transfer_id}, size={len(decoded_chunk)} bytes)"
        )

        if len(transfer["chunks"]) != transfer["total"]:
            self.get_logger().debug(
                f"[MISSION_FRAGMENT] Waiting for more fragments... "
                f"({len(transfer['chunks'])}/{transfer['total']} received)"
            )
            return

        # All fragments received, reassemble
        payload = b"".join(transfer["chunks"][index] for index in range(transfer["total"]))
        del self.mission_transfers[transfer_id]
        
        self.get_logger().info(
            f"[MISSION_FRAGMENT] All fragments received! Total payload: {len(payload)} bytes. Deserializing mission..."
        )
        self.publish_mission(payload)

    def publish_origin(self, data, return_address):

        origin = rp.OriginUpdateMessage.unpack(data)
        origin_msg = GeoPoint()
        origin_msg.latitude = origin.latitude
        origin_msg.longitude = origin.longitude
        origin_msg.altitude = origin.altitude
        self.origin_publisher.publish(origin_msg)
        self.get_logger().info(
            f"Published origin received over radio: lat={origin_msg.latitude}, "
            f"lon={origin_msg.longitude}, alt={origin_msg.altitude}"
        )
        response = rp.ConfirmOriginUpdateMessage(src_id=self.vehicle_id).pack()
        self.send_message(response, return_address)

    def publish_mission(self, data):
        try:
            if isinstance(data, (bytes, bytearray)):
                self.get_logger().info(f"[MISSION] Attempting to deserialize mission data: {len(data)} bytes")
                self.get_logger().info(f"[MISSION] Raw data start (hex): {data[:50].hex()}")
                mission_msg = deserialize_message(data, RouteNetwork)
            else:
                self.get_logger().info(f"[MISSION] Decoding base64 mission data")
                mission_data = base64.b64decode(data["data"])
                self.get_logger().info(f"[MISSION] Decoded mission data: {len(mission_data)} bytes")
                mission_msg = deserialize_message(mission_data, RouteNetwork)
            
            self.get_logger().info(
                f"[MISSION] RouteNetwork deserialized successfully"
            )
            self.get_logger().info(
                f"[MISSION] Number of waypoints: {len(mission_msg.points)}"
            )
            
            # Log detailed waypoint information
            if mission_msg.points:
                for idx, point in enumerate(mission_msg.points):
                    self.get_logger().info(
                        f"[MISSION] Waypoint {idx}: lat={point.position.latitude}, lon={point.position.longitude}, alt={point.position.altitude}"
                    )
            else:
                self.get_logger().warn(
                    f"[MISSION] WARNING: RouteNetwork has NO waypoints! "
                    f"Message structure: {mission_msg}"
                )
            
            self.mission_publisher.publish(mission_msg)
            self.get_logger().info(
                f"[MISSION] Published RouteNetwork with {len(mission_msg.points)} waypoints"
            )
        except Exception as error:
            self.get_logger().error(
                f"[MISSION] Failed to deserialize mission data: {error}"
            )
            self.get_logger().debug(f"[MISSION] Traceback: {traceback.format_exc()}")

    def handle_key_control(self, data):
        control_msg = rp.KeyControlMessage.unpack(data)

        if self.thruster_enable != control_msg.thruster_enabled:
            self.thruster_enable = control_msg.thruster_enabled
            enable = SetBool.Request()
            enable.data = self.thruster_enable
            self.e_kill_client.call_async(enable)

        ucommand_msg = UCommand()
        ucommand_msg.fin = [float(f) for f in control_msg.fin]
        ucommand_msg.thruster = control_msg.thruster if control_msg.thruster_enabled else 0
        self.ucommand_pub.publish(ucommand_msg)


    def init_vehicle(self, msg, return_address):
        self.get_logger().info("Initializing vehicle with received parameters")

        system_control_msg = rp.SystemControlMessage.unpack(msg)

        init_msg = SystemControl()
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.start.data = system_control_msg.start
        init_msg.rosbag_flag.data = system_control_msg.rosbag_flag
        init_msg.rosbag_prefix = system_control_msg.rosbag_prefix
        init_msg.thruster_arm.data = system_control_msg.thruster_arm
        init_msg.dvl_acoustics.data = system_control_msg.dvl_acoustics
        
        self.init_publisher.publish(init_msg)
        self.get_logger().info(f"Published INIT message with parameters: start={init_msg.start.data}, rosbag={init_msg.rosbag_flag.data}, thruster_arm={init_msg.thruster_arm.data}, dvl_acoustics={init_msg.dvl_acoustics.data}")
        
        # Send acknowledgment back to base station
        response = rp.ConfirmSystemControlMessage(src_id=self.vehicle_id).pack()
        self.send_message(response, return_address)

    
    def _apply_hardware_control(self, device, mode):
        """Runs set_relay.sh/set_strobe.sh for the given HardwareDevice/HardwareMode ints. Returns success bool."""
        script_path = self.HARDWARE_CONTROL_SCRIPTS.get(device)
        mode_name = self.HARDWARE_CONTROL_MODES.get(mode)
        device_name = "relay" if device == int(rp.HardwareDevice.RELAY) else "strobe"

        if script_path is None or mode_name is None:
            self.get_logger().error(f"Unknown hardware control device={device} mode={mode}")
            return False

        try:
            result = subprocess.run(
                ["bash", script_path, mode_name],
                capture_output=True,
                text=True,
                timeout=5,
            )
            success = result.returncode == 0
            if success:
                self.get_logger().info(f"Set {device_name} mode to '{mode_name}'")
            else:
                self.get_logger().error(
                    f"Failed to set {device_name} mode to '{mode_name}': {result.stderr.strip()}"
                )
            return success
        except Exception as e:
            self.get_logger().error(f"Error running {script_path}: {e}")
            return False

    def set_hardware_control(self, data, return_address):
        control_msg = rp.HardwareControlMessage.unpack(data)
        success = self._apply_hardware_control(control_msg.device, control_msg.mode)

        response = rp.ConfirmHardwareControlMessage(
            src_id=self.vehicle_id,
            device=control_msg.device,
            mode=control_msg.mode,
            success=success,
        ).pack()
        self.send_message(response, return_address)

    def hardware_control_topic_callback(self, msg):
        """Handles hardware control delivered directly over WiFi (not via XBee)."""
        try:
            device_str, mode_str = msg.data.split(":", 1)
            device = int(rp.HardwareDevice[device_str])
            mode = int(rp.HardwareMode[mode_str])
        except (ValueError, KeyError):
            self.get_logger().error(f"Ignoring malformed hardware control command: {msg.data}")
            return
        self._apply_hardware_control(device, mode)

    def kill_thruster(self, return_address):
        self.get_logger().info("Received kill command from base station")
        request = SetBool.Request()
        request.data = False

        while not self.e_kill_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the e_kill service. Exiting.")
                return
            self.get_logger().info("e_kill service not available, waiting again...")

        future = self.e_kill_client.call_async(request)

        def callback(fut, return_address):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info("Thruster has been deactivated.")
                else:
                    self.get_logger().error("Failed to deactivate thruster.")

                disarm_confirmed_msg = rp.ConfirmDisarmThrusterMessage(src_id=self.vehicle_id).pack()
                self.send_message(disarm_confirmed_msg, return_address)
            except Exception as e:
                self.get_logger().error(f"Error while trying to deactivate thruster: {str(e)}")

        future.add_done_callback(lambda fut: callback(fut, return_address))


    def destroy_node(self):
        self.running = False
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("XBee device closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RFBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down RF Bridge node.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
