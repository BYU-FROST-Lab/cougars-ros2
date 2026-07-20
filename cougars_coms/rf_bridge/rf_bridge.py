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
            self.get_logger().error("Failed to open XBee device")
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

    def battery_callback(self, msg):
        self.latest_battery = {
            "voltage": msg.voltage,
            "current": msg.current,
            "percentage": msg.percentage,
        }
        self.get_logger().debug("Updated battery data")

    def state_estimate_callback(self, msg):
        pose = msg.pose.pose
        self.latest_state_estimate = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }
        self.get_logger().debug("Updated state estimate data")

    def dvl_callback(self, msg):
        self.latest_dvl_velocity = {
            "x": msg.velocity.x,
            "y": msg.velocity.y,
            "z": msg.velocity.z,
            "velocity_valid": bool(msg.velocity_valid),
            "altitude": msg.altitude,
            "fom": msg.fom,
        }
        self.get_logger().debug("Updated DVL velocity data")

    def mission_feedback_callback(self, msg):
        self.latest_mission_feedback = {
            "state": msg.state,
            "elapsed_time": msg.elapsed_time,
            "waypoints_completed": msg.waypoints_completed,
            "waypoints_total": msg.waypoints_total,
            "mission_id": msg.mission_id,
        }
        self.get_logger().debug("Updated mission feedback data")

    def waypoint_feedback_callback(self, msg):
        self.latest_waypoint_feedback = {
            "state": msg.state,
            "horizontal_distance_error": msg.horizontal_distance_error,
            "depth_error": msg.depth_error,
            "bearing_error": msg.bearing_error,
        }
        self.get_logger().debug("Updated waypoint feedback data")
    
    def pressure_callback(self, msg):
        self.latest_pressure = {
            "fluid_pressure": msg.fluid_pressure,
            "variance": msg.variance,
        }
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
            status_response.x = self.latest_state_estimate["x"]
            status_response.y = self.latest_state_estimate["y"]
            status_response.depth = self.latest_state_estimate["z"]
            status_response.orientation_x = self.latest_state_estimate["qx"]
            status_response.orientation_y = self.latest_state_estimate["qy"]
            status_response.orientation_z = self.latest_state_estimate["qz"]
            status_response.orientation_w = self.latest_state_estimate["qw"]
        if self.latest_pressure != "NO_DATA":
            status_response.pressure = self.latest_pressure["fluid_pressure"]
            status_response.pressure_variance = self.latest_pressure["variance"]
        if self.latest_battery != "NO_DATA":
            status_response.battery_voltage = self.latest_battery["voltage"]
            status_response.battery_current = self.latest_battery["current"]
        if self.latest_dvl_velocity != "NO_DATA":
            status_response.dvl_velocity_x = self.latest_dvl_velocity["x"]
            status_response.dvl_velocity_y = self.latest_dvl_velocity["y"]
            status_response.dvl_velocity_z = self.latest_dvl_velocity["z"]
            status_response.dvl_altitude = self.latest_dvl_velocity["altitude"]
        if self.latest_waypoint_feedback != "NO_DATA":
            status_response.waypoint_state = self.latest_waypoint_feedback["state"]
            status_response.horizontal_distance_error = self.latest_waypoint_feedback["horizontal_distance_error"]
            status_response.depth_error = self.latest_waypoint_feedback["depth_error"]
            status_response.bearing_error = self.latest_waypoint_feedback["bearing_error"]
        if self.latest_mission_feedback != "NO_DATA":
            status_response.mission_id = self.latest_mission_feedback["mission_id"]
            status_response.mission_state = self.latest_mission_feedback["state"]
            status_response.waypoints_completed = self.latest_mission_feedback["waypoints_completed"]
            status_response.waypoints_total = self.latest_mission_feedback["waypoints_total"]
            status_response.elapsed_time = self.latest_mission_feedback["elapsed_time"]

        return status_response.pack()

    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data
            sender_id = payload[1] if len(payload) > 1 else None
            return_address = xbee_message.remote_device.get_64bit_addr()

            msg_id = payload[0] if len(payload) > 0 else None
            if isinstance(payload, (bytes, bytearray)) and payload.startswith(b"{\""):
                try:
                    decoded = json.loads(payload.decode("utf-8"))
                except Exception:
                    decoded = None
                if isinstance(decoded, dict) and decoded.get("message") == "MISSION_FRAGMENT":
                    self.handle_mission_fragment(decoded, return_address)
                    return
            if payload == b"MISSION_DONE":
                return
            if msg_id is not None:
                self.get_logger().info(f"Received message ID {msg_id} from {sender_id}")
            else:
                self.get_logger().info(f"Received message with no ID from {sender_id}")

            if msg_id not in [int(rp.MessageID.PING), int(rp.MessageID.STATUS_RESPONSE), int(rp.MessageID.CONFIRM_DISARM_THRUSTER), int(rp.MessageID.CONFIRM_SYSTEM_CONTROL)]:
                try:
                    self.publish_mission(payload)
                    return
                except Exception:
                    pass
            
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
                self.publish_origin(payload)
            # elif msg_id == int(rp.MessageID.MISSION):
            #     self.publish_mission(payload)
            # elif msg_id == int(rp.MessageID.KEY_CONTROL):
            #     self.get_logger().debug(f"Received KEY_CONTROL command {payload}")
            #     self.handle_key_control(payload)
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")
            # self.get_logger().error(traceback.format_exc())

    def handle_mission_fragment(self, fragment, return_address):
        transfer_id = fragment.get("id")
        chunk_index = fragment.get("i")
        total_chunks = fragment.get("n")
        data = fragment.get("data")

        if transfer_id is None or chunk_index is None or total_chunks is None or data is None:
            return

        transfer = self.mission_transfers.setdefault(transfer_id, {
            "chunks": {},
            "total": total_chunks,
            "address": return_address,
        })
        transfer["chunks"][chunk_index] = base64.b64decode(data)

        if len(transfer["chunks"]) != transfer["total"]:
            return

        payload = b"".join(transfer["chunks"][index] for index in range(transfer["total"]))
        del self.mission_transfers[transfer_id]
        self.publish_mission(payload)

    def publish_origin(self, data):

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
        self.send_message(response, origin_msg)

    def publish_mission(self, data):
        try:
            if isinstance(data, (bytes, bytearray)):
                mission_msg = deserialize_message(data, RouteNetwork)
            else:
                mission_data = base64.b64decode(data["data"])
                mission_msg = deserialize_message(mission_data, RouteNetwork)
            self.mission_publisher.publish(mission_msg)
            self.get_logger().info(
                f"Published RouteNetwork received over radio with "
                f"{len(mission_msg.points)} waypoints"
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to deserialize RouteNetwork: {error}"
            )

    def handle_key_control(self, msg):
        # self.get_logger().info(f"recieved key control through radio {msg}")
        ucommand_msg = UCommand()
        
        # Extract command data from nested structure
        command_data = msg.get("command", {})

        if self.thruster_enable != command_data.get('enable', False):
            self.thruster_enable = command_data.get('enable', False)
            enable = SetBool.Request()
            enable.data = self.thruster_enable
            self.e_kill_client.call_async(enable)

        
        # fin field expects an array of 4 floats
        fin_value = command_data.get("fin", [0.0, 0.0, 0.0, 0.0])
        fin_value[0] += 5
        # self.get_logger().info(f"fin value: {fin_value}")
        ucommand_msg.fin = [float(f) for f in fin_value]
        
        # Get throttle value from command data
        ucommand_msg.thruster = command_data.get("throttle", 0)
        # self.get_logger().info(f"thruster value: {ucommand_msg.thruster}")
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
