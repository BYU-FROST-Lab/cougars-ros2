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


def generate_launch_description():
    # default parameter file paths
    # TODO CHANGE THIS BACK
    param_file = (
        "/home/frostlab/config/deploy_tmp/vehicle_params.yaml"  # TODO CHANGE THIS BACK
    )
    fleet_param = "/home/frostlab/config/deploy_tmp/fleet_params.yaml"
    # Get the directory of the launch files
    cougars_control_package_dir = os.path.join(
        get_package_share_directory("cougars_control"), "launch"
    )

    namespace_launch_arg = DeclareLaunchArgument("namespace", default_value="coug0")
    sim_launch_arg = DeclareLaunchArgument("sim", default_value="False")
    param_file_launch_arg = DeclareLaunchArgument(
        "param_file", default_value=param_file
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        "fleet_param", default_value=fleet_param
    )
    verbose_launch_arg = DeclareLaunchArgument(
        "verbose",
        default_value="False",
    )

    launch_actions = []
    launch_actions.extend(
        [
            namespace_launch_arg,
            sim_launch_arg,
            param_file_launch_arg,
            fleet_param_launch_arg,
            verbose_launch_arg,
        ]
    )

    manual = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cougars_control_package_dir, "manual_launch.py")
        ),
        launch_arguments=[
            ("namespace", LaunchConfiguration("namespace")),
            ("sim", LaunchConfiguration("sim")),
            ("param_file", LaunchConfiguration("param_file")),
            ("fleet_param", LaunchConfiguration("fleet_param")),
            ("verbose", LaunchConfiguration("verbose")),
        ],
    )
    launch_actions.append(manual)

    # TODO add more nodes here that can be activated and disactivated

    launch_actions.extend(
        [
            launch_ros.actions.Node(
                package="cougars_bringup",
                executable="bag_recorder",
                name="bag_recorder",
                parameters=[
                    LaunchConfiguration("param_file"),
                    LaunchConfiguration("fleet_param"),
                ],
                namespace=LaunchConfiguration("namespace"),
                output="log",
            ),
            launch_ros.actions.Node(
                package="diagnostic_common_diagnostics",
                executable="cpu_monitor.py",
                # parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
                namespace=LaunchConfiguration("namespace"),
                output="log",
                remappings=[
                    ("/diagnostics", "diagnostics")
                ],  # remap to a relative topic that can be namespaced
            ),
            launch_ros.actions.Node(
                package="diagnostic_common_diagnostics",
                executable="ram_monitor.py",
                # parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
                namespace=LaunchConfiguration("namespace"),
                remappings=[
                    ("/diagnostics", "diagnostics")
                ],  # remap to a relative topic that can be namespaced
                output="log",
            ),
        ]
    )

    return launch.LaunchDescription(launch_actions)
