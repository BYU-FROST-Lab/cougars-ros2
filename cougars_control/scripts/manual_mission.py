#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cougars_interfaces.msg import (
    DesiredDepth,
    DesiredHeading,
    DesiredSpeed,
    SystemControl,
)
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


import time
import yaml


class ManualMission(Node):
    """
    :author: Braden Meyers
    :date: Apr 2025

    A simple ROS2 node that publishes desired depth, heading, and speed values to control the vehicle.
    The desired values are set based on a simple state machine that transitions between three states.

    Publishes:
        - desired_depth (cougars_interfaces/msg/DesiredDepth)
        - desired_heading (cougars_interfaces/msg/DesiredHeading)
        - desired_speed (cougars_interfaces/msg/DesiredSpeed)
        - system_status (indicate the termination of the mission)

    Services:
        - init (std_srvs/srv/Empty)
    """

    def __init__(self):
        """
        Creates a new ManualMission node.
        """
        super().__init__("manual_mission")

        # Default and initial parameters
        self.period = 0.5
        self.counter = 0
        self.start_time = time.time()
        self.state_index = 0
        self.started = False

        self.last_depth = -1.0
        self.last_heading = -1.0
        self.last_speed = -1.0

        # Declare parameters
        self.declare_parameter("vehicle_id", 0)

        self.declare_parameter("command_timer_period", self.period)  # in seconds
        """
        :param command_timer_period: The period at which the state machine updates the desired values. The default value is 0.5 seconds.
        """

        self.declare_parameter("mission_file_path", "")
        """
        :param states file path 
        """

        self.states = []  # Initialize as an empty list

        # Create the publishers
        self.depth_publisher = self.create_publisher(
            DesiredDepth, "desired_depth", qos_profile_system_default
        )
        """
        Publisher for the "desired_depth" topic with the message type DesiredDepth.
        """

        self.heading_publisher = self.create_publisher(
            DesiredHeading, "desired_heading", qos_profile_system_default
        )
        """
        Publisher for the "desired_heading" topic with the message type DesiredHeading.
        """

        self.speed_publisher = self.create_publisher(
            DesiredSpeed, "desired_speed", qos_profile_system_default
        )
        """
        Publisher for the "desired_speed" topic with the message type DesiredSpeed.
        """

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period")
            .get_parameter_value()
            .double_value,
            self.timer_callback,
        )
        """
        Timer that calls the timer_callback method at the specified period.
        """

        # Create the services
        self.srv = self.create_service(SetBool, "init_manual", self.listener_callback)
        """
        Service for the "init_manual" topic with the service type SetBool.
        """

        # Create the system services subscriber and publisher
        # TODO: Document
        self.system_status_pub = self.create_publisher(
            SystemControl, "system/status", 1
        )
        self.system_status_sub = self.create_subscription(
            SystemControl, "system/status", self.system_status_callback, 1
        )

        self.get_parameters()

    def load_states(self):
        # Load YAML file
        yaml_path = self.get_parameter("mission_file_path").value
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
                mission_type = data.get("mission_type", "")
                if mission_type == "manual":
                    states = data.get("states", [])
                    self.states = []

                    for i, state in enumerate(states):
                        state = {
                            "depth": state.get("depth"),
                            "heading": state.get("heading"),
                            "speed": state.get("speed"),
                            "time_seconds": state.get("time_seconds"),
                            "depth_from_bottom": state.get(
                                "depth_from_bottom", False
                            ),  # Default to False if not specified
                        }
                        self.states.append(state)
                    self.get_logger().info(
                        f"Loaded {len(self.states)} states from YAML"
                    )
                    return True
                else:
                    self.get_logger().info(
                        f"Did not load mission states because not right mission type"
                    )
                    return False
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.states = []

    def get_parameters(self):
        self.destroy_timer(self.timer)
        self.period = (
            self.get_parameter("command_timer_period")
            .get_parameter_value()
            .double_value
        )

        # Create a new timer with the updated period
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("Manual Mission Parameters Updated!")

    def system_status_callback(self, msg):
        """
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        """
        init_bool = msg.start.data
        if init_bool:
            if self.started:
                self.get_logger().info(
                    "Manual Mission has already been started. Needs to be reset before initialization"
                )
            else:
                start_mission = self.load_states()  # Load states from JSON file
                if start_mission:
                    self.get_parameters()
                    self.started = init_bool
                    self.get_logger().info("Manual Mission Started")
                    # self.counter = 0
                    self.start_time = time.time()
                    self.state_index = 0
                else:
                    self.get_logger().info("Mission type is not manual")
        else:
            self.started = False
            # self.counter = 0
            self.start_time = time.time()
            self.state_index = 0
            self.get_logger().info("Manual Mission Stopped")

    def listener_callback(self, request, response):
        # TODO: This will be deprecated soon
        """
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        """
        init_bool = request.data
        if init_bool:
            if self.started:
                response.success = False
                response.message = "Manual Mission has already been started. Needs to be reset before initialization"
            else:
                self.get_parameters()
                self.started = request.data
                response.success = True
                response.message = "Manual Mission Started"
                # self.counter = 0
                self.start_time = time.time()
        else:
            self.started = False
            # self.counter = 0
            self.start_time = time.time()
            response.success = True
            response.message = "Manual Mission Restarted"

        self.get_logger().info("this function is soon to be deprecated")

        return response

    def timer_callback(self):
        """
        Callback function for the timer.
        Runs the state machine and high-level controller, and publishes the desired depth, heading, and speed values.
        For a faster update time, adjust the command_timer_period parameter.
        """

        depth_msg = DesiredDepth()
        heading_msg = DesiredHeading()
        speed_msg = DesiredSpeed()

        # Elapsed time
        elapsed_time = time.time() - self.start_time
        # TODO actuallly start a timer instead of just counting ticks!!
        # Iterate through the states
        if self.states and self.started:
            if self.state_index < len(self.states):  # Technically this is redundant
                current_state = self.states[self.state_index]
                depth_msg.desired_depth = current_state["depth"]
                depth_msg.dfb = current_state["depth_from_bottom"]
                heading_msg.desired_heading = current_state["heading"]
                speed_msg.desired_speed = current_state["speed"]

                if elapsed_time < current_state["time_seconds"]:
                    # DO WE NEED ANYTHING HERE?
                    pass
                else:
                    # State time is up, move to the next state
                    self.state_index += (
                        1  # DO NOT REMOVE THIS LINE OR BAD THINGS WILL HAPPEN :)
                    )
                    # self.counter = 0.0  # Reset the counter for the next state
                    self.start_time = time.time()

                    # When completed all the states
                    if self.state_index == len(self.states):
                        # Reset the variables - This might be redundant because we have a subscriber
                        self.started = False
                        # self.counter = 0.0
                        self.start_time = time.time()
                        self.state_index = 0

                        msg = SystemControl()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.start.data = False
                        msg.rosbag_flag.data = False

                        self.system_status_pub.publish(msg)

            else:
                depth_msg.desired_depth = 0.0
                heading_msg.desired_heading = 0.0
                speed_msg.desired_speed = 0.0
        else:
            depth_msg.desired_depth = 0.0
            heading_msg.desired_heading = 0.0
            speed_msg.desired_speed = 0.0

        depth_msg.header.stamp = self.get_clock().now().to_msg()
        heading_msg.header.stamp = self.get_clock().now().to_msg()
        speed_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the messages
        self.depth_publisher.publish(depth_msg)
        self.heading_publisher.publish(heading_msg)
        self.speed_publisher.publish(speed_msg)

        # Publish the values if they change
        if (
            depth_msg.desired_depth != self.last_depth
            or heading_msg.desired_heading != self.last_heading
            or speed_msg.desired_speed != self.last_speed
        ):
            self.get_logger().info(
                "Depth: %f, Heading: %f, Speed: %f"
                % (
                    depth_msg.desired_depth,
                    heading_msg.desired_heading,
                    speed_msg.desired_speed,
                )
            )

        # Save last values
        self.last_depth = depth_msg.desired_depth
        self.last_heading = heading_msg.desired_heading
        self.last_speed = speed_msg.desired_speed


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
