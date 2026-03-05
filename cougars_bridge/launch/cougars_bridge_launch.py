import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

#########################################
# 
# Launches all the converter nodes that bridge
# the cougars hardward nodes with the algorithmic nodes
# Author: Clayton Smith
# 
#########################################


def generate_launch_description():

    # Declare launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value="coug0",
        description='Namespace for the vehicle'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value="/home/frostlab/config/vehicle_config.yaml",
        description='Path to the vehicle parameter file'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value="/home/frostlab/config/deploy_tmp/fleet_params.yaml",
        description='Path to the fleet parameter file'
    )
    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
    ]

  ### helper function to add nodes
    # Nodes all recieve the same parameter file, namespace,
    # and come with a turn-off condition
    def add_node(package:str, executable:str, default_on:bool=True, **kwargs):
        node_on = DeclareLaunchArgument(executable+'_on', default_value=default_on)
        node = launch_ros.actions.Node(
            package=package,
            executable=executable,
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration(executable+'_on')),
            **kwargs
        )
        launch_actions.extend(node_on, node)


  ### add ros nodes
    add_node(
        package='cougars_localization',
        executable='depth_converter'
    )
    add_node(
        package='cougars_localization',
        executable='dvl_converter'
    )
    add_node(
        package='cougars_localization',
        executable='dvl_global'
    )
    add_node(
        package='cougars_localization',
        executable='seatrac_ahrs_converter'
    )

    return launch.LaunchDescription(launch_actions)