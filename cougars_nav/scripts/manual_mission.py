#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from cougars_interfaces.msg import VehicleSetpoint, SystemControl
from rclpy.qos import qos_profile_system_default
from ament_index_python.packages import get_package_share_directory

import time
import yaml


class ManualMission(Node):
    '''
    :author: Braden Meyers
    :date: Apr 2026

    A simple ROS2 node that publishes desired depth, heading, and speed values to control the vehicle.
    The desired values are set based on a simple state machine that transitions between three states.

    Publishes:
        - guidance/setpoint_raw (VehicleSetpoint — DEPTH or ALTITUDE mode)
        - system/control (indicate the termination of the mission)
        
    '''
    def __init__(self):
        '''
        Creates a new ManualMission node.
        '''
        super().__init__("manual_mission")


        self.period = 0.5
        self.start_time = time.time()
        self.state_index = 0
        self.started = False

        self.last_depth = -1.0
        self.last_heading = -1.0
        self.last_speed = -1.0
        
        # Declare parameters
        self.declare_parameter('vehicle_id', 0)

        self.declare_parameter('command_timer_period', self.period) # in seconds

        cougars_nav_dir = os.path.join(get_package_share_directory('cougars_nav'))
        self.declare_parameter('mission_file_path', os.path.join(cougars_nav_dir, 'config', 'mission', 'test_mission.yaml'))

        
        self.states = []  # Initialize as an empty list

        # Create the publishers
        self.command_publisher = self.create_publisher(
            VehicleSetpoint,
            "guidance/setpoint_raw",
            qos_profile_system_default
        )

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period").get_parameter_value().double_value,
            self.timer_callback
        )


        # Create the system services subscriber and publisher
        self.system_ctrl_pub = self.create_publisher(SystemControl, 'system/control', 1)
        self.system_status_sub = self.create_subscription(SystemControl, 'system/control', self.system_control_callback, 1)

        self.get_parameters()

    
    def load_states(self):
        # Load YAML file
        yaml_path = self.get_parameter('mission_file_path').value
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                mission_type = data.get('mission_type', '')
                if mission_type == "manual":
                    states = data.get('states', [])
                    self.states = []

                    for state in states:
                        state = {
                            'depth': state.get('depth'),
                            'heading': state.get('heading'),  
                            'speed': state.get('speed'),    
                            'time_seconds': state.get('time_seconds'),
                            'depth_from_bottom': state.get('depth_from_bottom', False)  # Default to False if not specified
                        }
                        self.states.append(state)
                    self.get_logger().info(f"Loaded {len(self.states)} states from YAML")
                    return True
                else:
                    self.get_logger().info(f"Did not load mission states because not right mission type")
                    return False
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.states = []


    def get_parameters(self):
        self.destroy_timer(self.timer)
        self.period = self.get_parameter("command_timer_period").get_parameter_value().double_value

        # Create a new timer with the updated period
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("Manual Mission Parameters Updated!")

    
    def system_control_callback(self, msg):
        '''
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        '''
        init_bool = msg.start.data
        if init_bool:
            if self.started:
                self.get_logger().info('Manual Mission has already been started. Needs to be reset before initialization')
            else:
                start_mission = self.load_states()  # Load states from JSON file
                if start_mission:
                    self.get_parameters()
                    self.started = init_bool
                    self.get_logger().info('Manual Mission Started')
                    # self.counter = 0
                    self.start_time = time.time()
                    self.state_index = 0
                else:
                    self.get_logger().info('Mission type is not manual')
        else:
            self.started = False
            # self.counter = 0
            self.start_time = time.time()
            self.state_index = 0
            self.get_logger().info('Manual Mission Stopped')


    def timer_callback(self):
        msg = VehicleSetpoint()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.started and self.states and self.state_index < len(self.states):
            current_state = self.states[self.state_index]
            elapsed = time.time() - self.start_time

            msg.mode = VehicleSetpoint.ALTITUDE if current_state.get('depth_from_bottom', False) else VehicleSetpoint.DEPTH
            msg.depth = current_state['depth']
            msg.heading = current_state['heading']
            msg.speed = current_state['speed']
            msg.depth_valid = True
            msg.heading_valid = True
            msg.speed_valid = True

            if elapsed >= current_state['time_seconds']:
                self.state_index += 1
                self.start_time = time.time()

                if self.state_index >= len(self.states):
                    self.started = False
                    self.state_index = 0
                    done = SystemControl()
                    done.header.stamp = self.get_clock().now().to_msg()
                    done.start.data = False
                    done.rosbag_flag.data = False
                    self.system_ctrl_pub.publish(done)
                    self.get_logger().info('Mission complete')
        else:
            msg.mode = VehicleSetpoint.DEPTH
            msg.depth_valid = False
            msg.heading_valid = False
            msg.speed_valid = False

        self.command_publisher.publish(msg)

        if msg.depth != self.last_depth or msg.heading != self.last_heading or msg.speed != self.last_speed:
            self.get_logger().info(
                f'Setpoint — mode: {"ALT" if msg.mode == VehicleSetpoint.ALTITUDE else "DEPTH"}, '
                f'depth: {msg.depth:.2f}, heading: {msg.heading:.4f} rad, speed: {msg.speed:.2f} m/s'
            )
        self.last_depth = msg.depth
        self.last_heading = msg.heading
        self.last_speed = msg.speed


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    manual_mission = ManualMission()
    executor.add_node(manual_mission)
    executor.spin()
    executor.shutdown()
    manual_mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
