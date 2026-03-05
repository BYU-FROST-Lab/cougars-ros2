import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from modes import *

# 
# Top level launch file.
# Run this to start the robot.
# Calls other launch files.
# 


def generate_launch_description():
    
  ### Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0',
        description='Namespace for the vehicle'
    )
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml',
        description='Path to the vehicle parameter file'
    )
    
    fleet_param_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml',
        description='Path to the fleet parameter file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'modes',
        default_value='full',
        description=
        'Comma-separated list of system modes. ' \
        'Each mode specifies a list of launch configurations to set for . ' \
        'Modes are applied in order; ' \
        'earlier modes take precedence when settings conflict.' \
        'Existing modes include: full, sim, sensors, surface, demo'
    )

    flags_arg = DeclareLaunchArgument(
        'flags',
        default_value='',
        description=
        'Comma-separate list of launch argument overrides.' \
        'Takes precidence over modes.'
    )
    

  ### List of launch arguments to pass to branch launch files
    launch_args = [
        ('namespace',   LaunchConfiguration('namespace')),
        ('param_file',  LaunchConfiguration('param_file')),
        ('fleet_param', LaunchConfiguration('fleet_param')),
        ('modes',       LaunchConfiguration('modes')),
        ('flags',       LaunchConfiguration('flags'))
    ]

  ### Parse modes and flags to get the remaining launch arguments
    flags = parse_flags(launch_args['flags'])
    mode_list = parse_list(launch_args['modes'])
    resolved_args = resolve_modes(mode_list, flags)
    launch_args.extend(resolved_args.items())

  ### Package launch directories
    bridge_dir = os.path.join(
        get_package_share_directory('cougars_bridge'), 'launch')
    bringup_dir = os.path.join(
        get_package_share_directory('cougars_bringup'), 'launch')
    control_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')
    coms_dir = os.path.join(
        get_package_share_directory('cougars_coms'), 'launch')
    description_dir = os.path.join(
        get_package_share_directory('cougars_description'), 'launch')   
    localization_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')


  ### Launch files
    bridge_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_dir, "cougars_bridge_launch.py")),
        launch_arguments=launch_args)
    
    coms_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coms_dir, "coms_launch.py")),
        launch_arguments=launch_args)
    
    control_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, "control_launch.py")),
        launch_arguments=launch_args)
    
    waypoint_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, "waypoint_launch.py")),
        launch_arguments=launch_args)
    
    description_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, "cougars_description_launch.py")),
        launch_arguments=launch_args)
    
    localization_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, "cougars_localization_launch.py")),
        launch_arguments=launch_args)
    
    sensors_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "sensors_launch.py")),
        launch_arguments=launch_args)
    
    recorder_diagnostics_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "recorder_diagnostics_launch.py")),
        launch_arguments=launch_args)
    
    

  ### Launch actions
    launch_actions = [
        # launch arguments
        namespace_arg,
        param_file_arg,
        fleet_param_arg,
        mode_arg,
        flags_arg,

        # launch files
        bridge_launch,
        coms_launch,
        control_launch,
        waypoint_launch,
        description_launch,
        localization_launch,
        sensors_launch,
        recorder_diagnostics_launch
    ]

    return launch.LaunchDescription(launch_actions)