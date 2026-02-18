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

    # declare nodes
    depth_converter = launch_ros.actions.Node(
        package='cougars_localization',
        executable='depth_converter',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    dvl_converter = launch_ros.actions.Node(
        package='cougars_localization',
        executable='dvl_converter',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    dvl_global = launch_ros.actions.Node(
        package='cougars_localization',
        executable='dvl_global',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    seatrac_ahrs_convertor = launch_ros.actions.Node(
        package='cougars_localization',
        executable='seatrac_ahrs_convertor',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )


    # compile launch actions
    launch_actions = [

        # launch arguements
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,

        # launch nodes
        depth_converter,
        dvl_converter,
        dvl_global,
        seatrac_ahrs_convertor

    ]

    return launch.LaunchDescription(launch_actions)