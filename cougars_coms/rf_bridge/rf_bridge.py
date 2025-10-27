#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from dvl_msgs.msg import DVLDR
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from cougars_interfaces.msg import SystemControl, SystemStatus, UCommand

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import TransmitException
from pathlib import Path

import json
import threading
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
        self.latest_safety_status = "NO_DATA"
        self.latest_dvl_pos = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_depth = "NO_DATA"
        self.latest_pressure = "NO_DATA"

        self.thruster_enable = False

        self.vehicle_id = self.declare_parameter('vehicle_ID', 0).value
        self.base_station_id = self.declare_parameter('base_station_id', 15).value

        # XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/frost/xbee_radio').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)
        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")
            raise

        # ROS publishers and subscribers
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        self.init_publisher = self.create_publisher(SystemControl, 'system/status', 10)

        self.e_kill_client = self.create_client(SetBool, "arm_thruster")

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)

        self.subscription = self.create_subscription(
            SystemStatus,
            'safety_status',
            self.safety_status_callback,
            10)

        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery/data',
            self.battery_callback,
            10)
        
        self.dvl_sub = self.create_subscription(
            DVLDR,
            'dvl/position',
            self.dvl_callback,
            self.dvl_qos)
        
        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'depth_data',
            self.depth_callback,
            10)
        
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            'pressure/data',
            self.pressure_callback,
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

    def safe_numeric_convert(self, value, convert_func=float):
        """Safely convert ROS2 message fields to Python numeric types"""
        try:
            # Handle numpy/ROS2 integer types (int8, int16, etc.)
            if hasattr(value, 'item'):
                return convert_func(value.item())
            # Handle regular Python types
            else:
                return convert_func(value)
        except (ValueError, TypeError, AttributeError):
            return convert_func(0)  # Default fallback value

    def battery_callback(self, msg):
        self.latest_battery = {
            "voltage": self.safe_numeric_convert(msg.voltage, float),
        }
        self.get_logger().debug("Updated battery data")

    def safety_status_callback(self, msg):
        self.latest_safety_status = {
            "d_s": self.safe_numeric_convert(msg.depth_status, int),
            "g_s": self.safe_numeric_convert(msg.gps_status, int),
            "m_s": self.safe_numeric_convert(msg.modem_status, int),
            "d_s": self.safe_numeric_convert(msg.dvl_status, int),
            "e_s": self.safe_numeric_convert(msg.emergency_status, int)
        }
        self.get_logger().debug("Updated safety status data")

    def dvl_callback(self, msg):
        self.latest_dvl_pos = {
            "x": self.safe_numeric_convert(msg.position.x, float),
            "y": self.safe_numeric_convert(msg.position.y, float),
            "z": self.safe_numeric_convert(msg.position.z, float),
            "r": self.safe_numeric_convert(msg.roll, float),
            "p": self.safe_numeric_convert(msg.pitch, float),
            "y": self.safe_numeric_convert(msg.yaw, float)
        }
        self.get_logger().debug("Updated DVL position data")

    def depth_callback(self, msg):
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            pose = msg.pose.pose
            self.latest_depth = {
                "depth": self.safe_numeric_convert(pose.position.z, float)
            }
            self.get_logger().debug("Updated depth data")
    
    def pressure_callback(self, msg):
        self.latest_pressure = {
            "pres": self.safe_numeric_convert(msg.fluid_pressure, float),
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
            "s": self.latest_safety_status,
            "dv": self.latest_dvl_pos,
            "b": self.latest_battery,
            "d": self.latest_depth,
            "p": self.latest_pressure,
        }
        data_dict = {k: v for k, v in data_dict.items() if v and v != "NO_DATA"}
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
                self.kill_thruster()
            elif message_type == "INIT":
                init_msg = String()
                init_msg.data = "INIT_COMMAND"
                self.init_publisher.publish(init_msg)
                self.get_logger().info(f"Received INIT, published to init topic")
                self.device.send_data_broadcast("INIT_ACK")
            elif message_type == "KEY_CONTROL":
                self.get_logger().info(f"Received KEY_CONTROL command {data}")
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

    def handle_key_control(self, msg):
        self.get_logger().info(f"recieved key control through radio {msg}")
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
        self.get_logger().info(f"fin value: {fin_value}")
        ucommand_msg.fin = [float(f) for f in fin_value]
        
        # Get throttle value from command data
        ucommand_msg.thruster = command_data.get("throttle", 0)
        self.get_logger().info(f"thruster value: {ucommand_msg.thruster}")
        self.ucommand_pub.publish(ucommand_msg)

    
    def init_vehicle(self, msg):
        init_msg = SystemControl()
        init_msg.start = Bool(data=msg["start"])
        init_msg.rosbag_flag = Bool(data=msg["rosbag_flag"])
        init_msg.rosbag_prefix = msg["rosbag_prefix"]
        init_msg.thruster_arm = Bool(data=msg["thruster_arm"])
        init_msg.dvl_acoustics = Bool(data=msg["dvl_acoustics"])
        self.init_publisher.publish(init_msg)
        self.get_logger().info(f"Received INIT, published to init topic")
        self.device.send_data_broadcast(
                json.dumps({
                    "src_id": self.vehicle_id,
                    "message": "INIT"
                })
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
