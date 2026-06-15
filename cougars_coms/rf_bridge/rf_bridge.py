#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from dvl_msgs.msg import DVL
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from cougars_interfaces.msg import MissionFeedback, SystemControl, UCommand, WaypointFeedback
from geographic_msgs.msg import GeoPoint

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import TransmitException
from pathlib import Path

import json
import math
import traceback
import base64
import os
import time
import shutil

class RFBridge(Node):
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
            'state_estimate',
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

    def send_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().debug(f"Sent via XBee: {msg}")
        except TransmitException as e:
            pass
            # self.get_logger().error(f"XBee transmission error - TransmitException: {e}")
            # self.get_logger().error(traceback.format_exc())
        except Exception as e:
            pass
            # self.get_logger().error(f"XBee transmission error - Exception: {str(e)}")
            # self.get_logger().error(traceback.format_exc())

    def get_all_status_data(self):
        data_dict = {
            "src_id" : self.vehicle_id,
            "message" : "STATUS",
        }

        if self.latest_state_estimate != "NO_DATA":
            data_dict.update(self.latest_state_estimate)
        if self.latest_pressure != "NO_DATA":
            data_dict["pressure"] = self.latest_pressure["fluid_pressure"]
            data_dict["pressure_variance"] = self.latest_pressure["variance"]
        if self.latest_battery != "NO_DATA":
            data_dict.update({
                "voltage": self.latest_battery["voltage"],
                "current": self.latest_battery["current"],
                "percentage": self.latest_battery["percentage"],
            })
        if self.latest_dvl_velocity != "NO_DATA":
            data_dict.update({
                "dvl_x": self.latest_dvl_velocity["x"],
                "dvl_y": self.latest_dvl_velocity["y"],
                "dvl_z": self.latest_dvl_velocity["z"],
                "dvl_valid": self.latest_dvl_velocity["velocity_valid"],
                "dvl_altitude": self.latest_dvl_velocity["altitude"],
                "dvl_fom": self.latest_dvl_velocity["fom"],
            })
        if self.latest_waypoint_feedback != "NO_DATA":
            data_dict.update({
                "ws": self.latest_waypoint_feedback["state"],
                "hd": self.latest_waypoint_feedback["horizontal_distance_error"],
                "de": self.latest_waypoint_feedback["depth_error"],
                "be": self.latest_waypoint_feedback["bearing_error"],
            })
        if self.latest_mission_feedback != "NO_DATA":
            data_dict.update({
                "mission_id": self.latest_mission_feedback["mission_id"],
                "ms": self.latest_mission_feedback["state"],
                "wc": self.latest_mission_feedback["waypoints_completed"],
                "tw": self.latest_mission_feedback["waypoints_total"],
                "et": self.latest_mission_feedback["elapsed_time"],
            })

        data_dict = {
            key: value for key, value in data_dict.items()
            if not isinstance(value, float) or math.isfinite(value)
        }
        return json.dumps(data_dict, separators=(',', ':'))

    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            return_address = xbee_message.remote_device.get_64bit_addr()
            self.get_logger().debug(f"Received from {return_address}: {payload}")
            
            msg = String()
            msg.data = payload
            self.publisher.publish(msg)

            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                data = payload  # If JSON decoding fails, treat payload as a string

            if isinstance(data, dict):
                message_type = data.get("message")
            else:
                message_type = data
            
            self.get_logger().debug(f"Published message: {payload}")
            if message_type == "STATUS":
                response = self.get_all_status_data()
                self.send_message(response, return_address)
                self.get_logger().debug(f"Received STATUS, responding with sensor data")
                self.get_logger().debug(f"Status response: {response}")
            elif message_type == "PING":
                response = {"message": "PING", "src_id": self.vehicle_id}
                self.send_message(json.dumps(response), return_address)
                self.get_logger().debug(f"Received PING, responding with PING")
            elif message_type == "E_KILL":
                self.get_logger().info(f"Received E_KILL message")
                self.kill_thruster(return_address)
            elif message_type == "INIT":
                self.get_logger().info(f"Received INIT command {data}")
                self.init_vehicle(data, return_address)
            elif message_type == "ORIGIN":
                self.publish_origin(data)
            elif message_type == "KEY_CONTROL":
                self.get_logger().debug(f"Received KEY_CONTROL command {data}")
                self.handle_key_control(data)
            elif message_type == "FILE_START":
                self.handle_file_start(data, return_address)
            elif message_type == "FILE_CHUNK":
                self.handle_file_chunk(data, return_address)
            elif message_type == "FILE_END":
                self.handle_file_end(data, return_address)
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")
            # self.get_logger().error(traceback.format_exc())

    def publish_origin(self, data):
        origin_msg = GeoPoint()
        origin_msg.latitude = float(data["lat"])
        origin_msg.longitude = float(data["lon"])
        origin_msg.altitude = float(data["alt"])
        self.origin_publisher.publish(origin_msg)
        self.get_logger().info(
            f"Published origin received over radio: lat={origin_msg.latitude}, "
            f"lon={origin_msg.longitude}, alt={origin_msg.altitude}"
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
        init_msg = SystemControl()
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.start.data = msg.get("start", False)
        init_msg.rosbag_flag.data = msg.get("rosbag_flag", False)
        init_msg.rosbag_prefix = msg.get("rosbag_prefix", "")
        init_msg.thruster_arm.data = msg.get("thruster_arm", False)
        init_msg.dvl_acoustics.data = msg.get("dvl_acoustics", False)
        
        self.init_publisher.publish(init_msg)
        self.get_logger().info(f"Published INIT message with parameters: start={init_msg.start.data}, rosbag={init_msg.rosbag_flag.data}, thruster_arm={init_msg.thruster_arm.data}, dvl_acoustics={init_msg.dvl_acoustics.data}")
        
        # Send acknowledgment back to base station
        response = {
            "src_id": self.vehicle_id,
            "message": "INIT_ACK",
            "success": True,
            "status": "received",
        }
        self.send_message(
            json.dumps(response, separators=(',', ':')),
            return_address,
        )

    
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

                msg_dict = {
                    "src_id": self.vehicle_id,  
                    "message": "E_KILL",
                    "success": response.success
                }
                msg = json.dumps(msg_dict)
                self.send_message(msg, return_address)

            except Exception as e:
                self.get_logger().error(f"Error while trying to deactivate thruster: {str(e)}")

        future.add_done_callback(lambda fut: callback(fut, return_address))

    def handle_file_start(self, data, sender_address):
        """Handle FILE_START message to initialize file transfer"""
        try:
            transfer_id = data.get("transfer_id")
            filename = data.get("filename")
            total_chunks = data.get("total_chunks")
            file_size = data.get("file_size")
            
            self.get_logger().info(f"Starting file transfer: {filename} ({file_size} bytes, {total_chunks} chunks)")
            
            # Initialize transfer state
            self.active_transfers[transfer_id] = {
                "filename": filename,
                "total_chunks": total_chunks,
                "file_size": file_size,
                "sender_address": sender_address,
                "received_chunks": 0,
                "start_time": time.time()
            }
            
            # Initialize chunk storage
            self.file_chunks[transfer_id] = {}
            
        except Exception as e:
            self.get_logger().error(f"Error handling FILE_START: {e}")
            self.send_file_ack(sender_address, "error", f"Failed to initialize transfer: {str(e)}")

    def handle_file_chunk(self, data, sender_address):
        """Handle FILE_CHUNK message to receive file data"""
        try:
            transfer_id = data.get("transfer_id")
            chunk_num = data.get("chunk_num")
            chunk_data = data.get("data")
            
            if transfer_id not in self.active_transfers:
                self.get_logger().warn(f"Received chunk for unknown transfer {transfer_id}")
                return
            
            # Decode base64 data
            chunk_bytes = base64.b64decode(chunk_data)
            
            # Store chunk
            self.file_chunks[transfer_id][chunk_num] = chunk_bytes
            self.active_transfers[transfer_id]["received_chunks"] += 1
            
            received = self.active_transfers[transfer_id]["received_chunks"]
            total = self.active_transfers[transfer_id]["total_chunks"]
            
            self.get_logger().debug(f"Received chunk {chunk_num + 1}/{total} for transfer {transfer_id}")
            
            # Check if we have all chunks
            if received == total:
                self.get_logger().info(f"All chunks received for transfer {transfer_id}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling FILE_CHUNK: {e}")
            self.send_file_ack(sender_address, "error", f"Failed to process chunk: {str(e)}")

    def handle_file_end(self, data, sender_address):
        """Handle FILE_END message to finalize file transfer"""
        try:
            transfer_id = data.get("transfer_id")
            filename = data.get("filename")
            
            if transfer_id not in self.active_transfers:
                self.get_logger().warn(f"Received FILE_END for unknown transfer {transfer_id}")
                return
            
            transfer_info = self.active_transfers[transfer_id]
            
            # Check if we have all chunks
            if len(self.file_chunks[transfer_id]) != transfer_info["total_chunks"]:
                missing_chunks = transfer_info["total_chunks"] - len(self.file_chunks[transfer_id])
                error_msg = f"Missing {missing_chunks} chunks"
                self.get_logger().error(error_msg)
                self.send_file_ack(sender_address, "error", error_msg)
                return
            
            # Reassemble file
            file_data = bytearray()
            for chunk_num in sorted(self.file_chunks[transfer_id].keys()):
                file_data.extend(self.file_chunks[transfer_id][chunk_num])
            
            # Save file
            file_path = os.path.join(self.received_files_dir, filename)
            with open(file_path, 'wb') as f:
                f.write(file_data)
            
            elapsed_time = time.time() - transfer_info["start_time"]
            self.get_logger().info(f"File transfer complete: {filename} saved to {file_path} ({len(file_data)} bytes in {elapsed_time:.2f}s)")
            
            # Send success acknowledgment
            self.send_file_ack(sender_address, "complete", transfer_id)
            
            # Process the mission file if it's a mission
            if filename == self.MISSION_FILE:
                self.process_received_mission(file_path)
            elif filename == self.FLEET_PARAMS_FILE:
                self.process_received_fleet_params(file_path)
            elif filename == self.VEHICLE_PARAMS_FILE:
                self.process_received_vehicle_params(file_path)
            else:
                self.get_logger().info(f"Received unrecognized file: {filename}, saved to {file_path}")

            # Clean up transfer state
            del self.active_transfers[transfer_id]
            del self.file_chunks[transfer_id]
            
        except Exception as e:
            self.get_logger().error(f"Error handling FILE_END: {e}")
            self.send_file_ack(sender_address, "error", f"Failed to finalize transfer: {str(e)}")

    def send_file_ack(self, sender_address, status, transfer_id_or_error):
        """Send file transfer acknowledgment to base station"""
        try:
            ack_msg = {
                "message": "FILE_ACK",
                "src_id": self.vehicle_id,
                "status": status,
                "transfer_id": transfer_id_or_error if status == "complete" else None,
                "error": transfer_id_or_error if status == "error" else None
            }
            self.send_message(json.dumps(ack_msg), sender_address)
        except Exception as e:
            self.get_logger().error(f"Failed to send file ACK: {e}")

    def process_received_mission(self, received_mission_path):
        """Process a received mission file"""
        try:
            # Copy mission file to the expected location
            mission_file_path = Path("/home/frostlab/config/deploy_tmp/mission.yaml")

            shutil.copy2(received_mission_path, mission_file_path)

            self.get_logger().info(f"Mission file processed and copied to {mission_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission file: {e}")

    def process_received_fleet_params(self, received_fleet_params_path):
        """Process a received fleet params file"""
        try:
            # Copy fleet params file to the expected location
            fleet_params_path = Path("/home/frostlab/config/deploy_tmp/fleet_params.yaml")  # Adjust this path as needed for your system

            shutil.copy2(received_fleet_params_path, fleet_params_path)

            self.get_logger().info(f"Fleet params file processed and copied to {fleet_params_path}")

        except Exception as e:
            self.get_logger().error(f"Error processing fleet params file: {e}")

    def process_received_vehicle_params(self, received_vehicle_params_path):
        """Process a received vehicle params file"""
        try:
            # Copy vehicle params file to the expected location
            vehicle_params_path = Path(f"/home/frostlab/config/deploy_tmp/coug{self.vehicle_id}_params.yaml")

            shutil.copy2(received_vehicle_params_path, vehicle_params_path)

            self.get_logger().info(f"Vehicle params file processed and copied to {vehicle_params_path}")

        except Exception as e:
            self.get_logger().error(f"Error processing vehicle params file: {e}")

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
