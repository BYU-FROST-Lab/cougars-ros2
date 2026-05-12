import sys

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        default_value='/home/frostlab/config/agent/vehicle_params.yaml'
    )
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/fleet/fleet_params.yaml'
    )

    ### Launch Nodes
    use_sim_time = LaunchConfiguration('use_sim_time')

    mission_publisher = launch_ros.actions.Node(
        package='cougars_bringup',
        executable='mission_publisher.py',
        name='mission_publisher',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            LaunchConfiguration('param_file'),
            LaunchConfiguration('fleet_param'),
            {'use_sim_time': use_sim_time}
        ],
        namespace=LaunchConfiguration('namespace'),
        output='log',
    )
    origin_publisher = launch_ros.actions.Node(
        package='cougars_bringup',
        executable='origin_publisher.py',
        name='origin_publisher',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param'), {'use_sim_time': use_sim_time}],
        namespace=LaunchConfiguration('namespace'),
        output='log',
    )
    bag_recorder = launch_ros.actions.Node(
        package='cougars_bringup',
        executable='bag_recorder',
        name='bag_recorder',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param'), {'use_sim_time': use_sim_time}],
        namespace=LaunchConfiguration('namespace'),
        output='log',
    )
    cpu_monitor = launch_ros.actions.Node(
        package='diagnostic_common_diagnostics',
        executable='cpu_monitor.py', 
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
        remappings=[('/diagnostics', 'diagnostics')],   # remap to a relative topic that can be namespaced
    )
    ram_monitor = launch_ros.actions.Node(
        package='diagnostic_common_diagnostics',
        executable='ram_monitor.py',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/diagnostics', 'diagnostics')],    # remap to a relative topic that can be namespaced
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_sim_time_launch_arg,
        OpaqueFunction(function=debug_launch_args),

        # launch nodes
        mission_publisher,
        origin_publisher,
        bag_recorder,
        cpu_monitor,
        ram_monitor
    ]

    return launch.LaunchDescription(launch_actions)


def debug_launch_args(context, *args, **kwargs):
    sim_val = LaunchConfiguration('sim').perform(context)
    print(f"[DEBUG] sim = {sim_val}")
    return []