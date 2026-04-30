import sys
import rclpy
import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
BAGNAME = '3.0_SO-2025-05-01-19-07-06/'
BAGDIR="/home/frostlab/bag/"
NS='/coug1'
RECORD_ROSBAG = False

def generate_launch_description():
    '''

    Launches the manual control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    verbose = "true"  # Default to 'false'

    namespace = NS

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'
    
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    launch_actions = []
    #sets namespace of ekf node - unfortunately neccisary bc yamls can't be dynamically configured as far as I know
    # with open('/home/frostlab/ros2_ws/config/ekf.yaml', 'r') as f:
    #     #change line containing 'ekf node' to 'NS/ekf_node'
    #     lines=f.readlines()
    #     for line in range(len(lines)):
    #         if 'ekf_node' in lines[line]:
    #             lines[line]=NS+'/ekf_node:\n'
    # with open('/home/frostlab/ros2_ws/config/ekf.yaml', 'w') as f:
    #     f.writelines(lines)
    
    launch_actions.extend([
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            namespace=namespace,
            output=output
        ),
        launch.actions.ExecuteProcess(
            cmd=["ros2", "bag", "play", BAGDIR+BAGNAME, "--clock", "-r 2"], #r = rate, was at 5
            output=output
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='dummy_factor_starter.py',
            namespace=namespace,
            output=output
        )
        # ,launch_ros.actions.Node(
        # package='robot_localization',
        # executable='ekf_node',
        # name='ekf_node',
        # output=output,
        # namespace=namespace,
        # parameters=['/home/frostlab/ros2_ws/config/ekf.yaml']
        # )
    ])
    if(RECORD_ROSBAG):
        launch_actions.extend([
        launch.actions.TimerAction(
            period=2.0, #record after 2 seconds
            actions= [launch.actions.ExecuteProcess(cmd=["ros2", "bag", "record", NS+"/smoothed_output"],output=output)]
        )]
        )
    return launch.LaunchDescription(launch_actions)