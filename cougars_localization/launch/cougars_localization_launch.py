import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # TODO: add localization

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
    ])
