#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cougars_interfaces.msg import UCommand
from std_srvs.srv import SetBool
import ast
from rclpy.qos import qos_profile_system_default


class FinsManual(Node):
    '''
    :author: Braden Meyers, Nelson Durrant
    :date: January 2025

    A ROS2 node that publishes to controls/command for control surface commands manual defined from parameters

    Publishes:
        - controls/command (cougars_interfaces/msg/UCommand)
        
    Services:
        - init_manual (std_srvs/srv/SetBool) 
    '''
    def __init__(self):
        '''
        Creates a new ManualMission node.
        '''
        super().__init__("fins_manual")


        self.declare_parameter('command_timer_period', 0.5) # in seconds
        '''
        :param command_timer_period: The period at which the state machine updates the desired values. The default value is 0.5 seconds.
        '''

        self.declare_parameter("commands", "[]")
        '''
        :param commands: a string of commands that has lists of length 5 [count, fin1, fin2, fin3, thruster] ie. commands: "[10, 15, 15, 15, 0], [10, -15, -15, -15, 0], [10, 25, 25, 25, 0]"
        '''
        commands = self.get_parameter('commands').value
        self.commands = ast.literal_eval(commands)
        self.command_index = 0
        print("Got commands:", self.commands)

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period").get_parameter_value().double_value,
            self.timer_callback
        )
        '''
        Timer that calls the timer_callback method at the specified period.
        '''

        # Create the publishers
        self.controls_publisher = self.create_publisher(
            UCommand,
            "controls/command",
            qos_profile_system_default
        )
        '''
        Publisher for the "desired_depth" topic with the message type DesiredDepth.
        '''

        self.manual_srv = self.create_service(
            SetBool,
            "init_manual",
            self.init_callback
        )
        '''
        Service for the "init_manual" topic with the service type SetBool.
        '''

        self.counter = 0
        self.started = False

        self.last_command = -1


    def init_callback(self, request, response):
        '''
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        '''

        #TODO restart doestn work
        init_bool = request.data
        if init_bool:
            if self.started:
                response.success = False
                response.message = 'Fin Mission has already been started. Needs to be reset before initialization'
            else:
                self.started = request.data
                response.success = True
                response.message = 'Fin Mission Started'
        else:
            self.started = False
            self.counter = 0
            response.success = True
            response.message = 'Fin Mission Restarted'

        return response


    def timer_callback(self):
        '''
        Callback function for the timer.
        Publishes the commands in the variable self.commands. Publishes for number of iterations specified in count  
        For a faster update time, adjust the command_timer_period parameter.
        '''

        command_msg = UCommand()

        if self.started and (self.command_index < len(self.commands)):
            if self.commands[self.command_index][0] <= self.counter:
                self.command_index += 1
                if self.command_index >= len(self.commands):
                    return
                self.counter = 0
            command_msg.fin[0] = self.commands[self.command_index][1] 
            command_msg.fin[1] = self.commands[self.command_index][2] 
            command_msg.fin[2] = self.commands[self.command_index][3] 
            command_msg.thruster = self.commands[self.command_index][4] 

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

            # Publish the values if they change
            if self.command_index != self.last_command:
                self.get_logger().info("Sent Command: Count - %d, Fin1: %d, Fin2: %d, Fin3: %d, Thruster: %d" % (
                    self.commands[self.command_index][0],
                    self.commands[self.command_index][1],
                    self.commands[self.command_index][2],
                    self.commands[self.command_index][3],
                    self.commands[self.command_index][4],
                    )
                )
                # Save last values
                self.last_command = self.command_index


        #Add Time stamp to message
        command_msg.header.stamp = self.get_clock().now().to_msg()
        # Publish the messages
        self.controls_publisher.publish(command_msg)
            


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    manual_fins = FinsManual()
    executor.add_node(manual_fins)
    executor.spin()
    executor.shutdown()
    manual_fins.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()