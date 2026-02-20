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
# Top level launch file.
# Run this to start the robot.
# Calls other launch files.
# 


def generate_launch_description():
    
  ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0')
    
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False')
    
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml')
    
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml')
    
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False')
    


  ### Launch arguments list
    launch_arguments = [
        ('namespace', LaunchConfiguration('namespace')),
        ('sim', LaunchConfiguration('sim')),
        ('param_file', LaunchConfiguration('param_file')),
        ('fleet_param', LaunchConfiguration('fleet_param')),
        ('verbose', LaunchConfiguration('verbose')),
    ]


  ### Package launch directories
    cougars_bridge_dir = os.path.join(
        get_package_share_directory('cougars_bridge'), 'launch')
    cougars_bringup_dir = os.path.join(
        get_package_share_directory('cougars_bringup'), 'launch')
    cougars_control_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')
    cougars_coms_dir = os.path.join(
        get_package_share_directory('cougars_coms'), 'launch')
    cougars_description_dir = os.path.join(
        get_package_share_directory('cougars_description'), 'launch')   
    cougars_localization_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')



  ### Launch files
    bridge_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_bridge_dir, "cougars_bridge_launch.py")),
        launch_arguments=launch_arguments)
    
    coms_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_coms_dir, "coms_launch.py")),
        launch_arguments=launch_arguments)
    
    control_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_control_dir, "control_launch.py")),
        launch_arguments=launch_arguments)
    
    waypoint_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_control_dir, "waypoint_launch.py")),
        launch_arguments=launch_arguments)
    
    description_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_description_dir, "cougars_description_launch.py")),
        launch_arguments=launch_arguments)
    
    localization_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_localization_dir, "cougars_localization_launch.py")),
        launch_arguments=launch_arguments)
    
    sensors_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_bringup_dir, "sensors_launch.py")),
        launch_arguments=launch_arguments)
    
    recorder_diagnostics_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_bringup_dir, "recorder_diagnostics_launch.py")),
        launch_arguments=launch_arguments)
    
    

  ### Launch actions
    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        sim_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        verbose_launch_arg,

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