# Launch file for the RF bridge node in the cougars_coms package


import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=f'{Path.home()}/config/agent/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=f'{Path.home()}/config/fleet/fleet_params.yaml'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Use simulation clock if true'
    )
    
    acoustic_ping_arg = DeclareLaunchArgument(
        'acoustic_pinger',
        default_value='false',
        description='enables the vehicle pinger which will send acoustic messages on repeat'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )

    # Get the parameter values
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    fleet_param = LaunchConfiguration('fleet_param')
    sim = LaunchConfiguration('sim')
    acoustic_ping = LaunchConfiguration('acoustic_pinger')
    debug = LaunchConfiguration('debug')

    # Define the nodes
    cougars_coms_node = Node(
        package='cougars_coms',
        executable='cougars_coms',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace
    )

    rf_bridge_node = Node(
        package='cougars_coms',
        executable='rf_bridge.py',
        name='rf_bridge',
        namespace=namespace,
        parameters=[
            param_file, fleet_param,
            {'debug_mode': debug},
            {'use_sim_time': sim}
        ],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(sim)
    )

    vehicle_pinger_node = Node(
        package='cougars_coms',
        executable='vehicle_pinger',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        condition=IfCondition(acoustic_ping),
    )


    launch_actions = [
        # launch args
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        sim_launch_arg,
        acoustic_ping_arg,
        debug_arg,

        # nodes
        cougars_coms_node,
        rf_bridge_node,
        vehicle_pinger_node
    ]

    # Return the launch description
    return LaunchDescription(launch_actions)