import launch
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

import os

def generate_launch_description():
    '''
    Launches the basic control nodes for the vehicle.
    Parameters:
    - fins_manual: if true, use fins_manual.py instead of coug_controls.cpp
    - manual_mission: if true, run manual_mission.py instead of waypoint_follower.cpp. fins_manual and manual_mission are NOT related.  fins_manual directly controls the fins, while manual_mission runs a timed mission.
    '''

  ### launch args
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )
    demo_launch_arg = DeclareLaunchArgument(
        'demo',
        default_value='False'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/agent/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/fleet/fleet_params.yaml'
    )
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False',    
    )
    fins_manual_launch_arg = DeclareLaunchArgument(
        'fins_manual',
        default_value='False'
    )
    manual_mission_launch_arg = DeclareLaunchArgument(
        'manual_mission',
        default_value='False'
    )

  ### Get launch argument values
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    fleet_param = LaunchConfiguration('fleet_param')
    use_sim_time = LaunchConfiguration('use_sim_time')


  ### Node definitions

    # initialize communication with teensy
    mc_serial_node = Node(
        package='cougars_control',
        executable='mc_serial_node',
        parameters=[param_file, fleet_param, {'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='log',
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # kinematics node converts control inputs to fin outputs
    kinematics_node = Node(
        package='cougars_control',
        executable='coug_kinematics',
        parameters=[param_file, fleet_param, {'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='log',
    )

    controls_node = Node(
        package='cougars_control',
        executable='coug_controls',
        parameters=[param_file, fleet_param, {'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='log',
        condition=UnlessCondition(LaunchConfiguration('fins_manual'))
    )

    # Fins manual node — if fins_manual is true, replaces controls_node
    fins_manual_node = Node(
        package='cougars_control',
        executable='fins_manual.py',
        parameters=[param_file, fleet_param, {'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='log',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('fins_manual'))
    )



    launch_actions = [
        # launch args
        namespace_launch_arg,
        sim_launch_arg,
        use_sim_time_launch_arg,
        demo_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        fins_manual_launch_arg,
        manual_mission_launch_arg,
        verbose_launch_arg,

        # nodes
        mc_serial_node,
        kinematics_node,
        controls_node,
        fins_manual_node
    ]

    return launch.LaunchDescription(launch_actions)
