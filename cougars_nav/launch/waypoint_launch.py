import launch
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def debug_launch_args(context, *args, **kwargs):
    sim_val = LaunchConfiguration('sim').perform(context)
    print(f"[DEBUG] sim = {sim_val}")
    return []

def generate_launch_description():
    '''
    Launches the waypoint navigation nodes for the vehicle.
    '''

    # default parameter file paths
    param_file = '/home/frostlab/config/agent/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/fleet/fleet_params.yaml'

    # launch parameters
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    # if sim is true, include demo launch
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=param_file
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=fleet_param
    )
    
    launch_actions = []
    launch_actions.extend([
        namespace_launch_arg,
        sim_launch_arg,
            use_sim_time_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        OpaqueFunction(function=debug_launch_args)
    ])

    waypoint_iterator = Node(
        package='cougars_nav',
        executable='waypoint_iterator',
        name='waypoint_iterator',
        namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('param_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    waypoint_controller = Node(
        package='cougars_nav',
        executable='waypoint_controller',
        name='waypoint_controller',
        namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('param_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('state_estimate', '/holoocean/auv0/DynamicsSensorOdom')],
        output='screen',
    )

    launch_actions.append(waypoint_iterator)
    launch_actions.append(waypoint_controller)

    return launch.LaunchDescription(launch_actions)
