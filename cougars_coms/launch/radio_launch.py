# Launch file for the RF bridge node in the cougars_coms package


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="coug0",
        description="Vehicle namespace (e.g., coug0)",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value="/home/frostlab/config/vehicle_params.yaml",
        description="Path to vehicle parameters YAML file",
    )

    debug_arg = DeclareLaunchArgument(
        "debug", default_value="false", description="Enable debug logging"
    )

    # Get the parameter values
    namespace = LaunchConfiguration("namespace")
    param_file = LaunchConfiguration("param_file")
    debug = LaunchConfiguration("debug")

    # Define the node
    rf_bridge_node = Node(
        package="cougars_coms",
        executable="rf_bridge.py",
        name="rf_bridge",
        namespace=namespace,
        parameters=[param_file, {"debug_mode": debug}],
        output="screen",
        emulate_tty=True,
    )

    # Return the launch description
    return LaunchDescription([namespace_arg, param_file_arg, debug_arg, rf_bridge_node])
