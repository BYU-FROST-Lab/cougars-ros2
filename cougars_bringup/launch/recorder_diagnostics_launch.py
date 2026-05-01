import sys

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

# 
# Runs the rec
# 


def generate_launch_description():

  ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml'
    )

  ### Launch Nodes
    bag_recorder = launch_ros.actions.Node(
        package='cougars_bringup',
        executable='bag_recorder',
        name='bag_recorder',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')], 
        namespace=LaunchConfiguration('namespace'),
        output='log',
    )
    cpu_monitor = launch_ros.actions.Node(
        package='diagnostic_common_diagnostics',
        executable='cpu_monitor.py',
        # parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')], 
        namespace=LaunchConfiguration('namespace'),
        output='log',
        remappings=[('/diagnostics', 'diagnostics')],   # remap to a relative topic that can be namespaced
    )
    ram_monitor = launch_ros.actions.Node(
        package='diagnostic_common_diagnostics',
        executable='ram_monitor.py',
        # parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')], 
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/diagnostics', 'diagnostics')],    # remap to a relative topic that can be namespaced
        output='log',
    )

    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,

        # launch nodes
        bag_recorder,
        cpu_monitor,
        ram_monitor
    ]

    return launch.LaunchDescription(launch_actions)