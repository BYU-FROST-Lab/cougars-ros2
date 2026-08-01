# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    OrSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs) -> list:
    sim = LaunchConfiguration("sim")
    namespace = LaunchConfiguration("namespace")
    fleet_params = LaunchConfiguration('fleet_param')
    auv_params = LaunchConfiguration('param_file')

    urdf_filename = "couguv.urdf.xacro"

    coug_description_dir = get_package_share_directory("coug_description")
    urdf_file = os.path.join(coug_description_dir, "urdf", urdf_filename)

    frame_prefix = PythonExpression(["'", namespace, "/' if '", namespace, "' != '' else ''"])

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                fleet_params,
                auv_params,
                {
                    "robot_description": ParameterValue(
                        Command(["xacro ", urdf_file]),
                        value_type=str,
                    ),
                    "use_sim_time": sim,
                    "frame_prefix": frame_prefix,
                },
            ],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                fleet_params,
                auv_params,
                {"use_sim_time": sim},
            ],
            condition=IfCondition(
                OrSubstitution(
                    NotEqualsSubstitution(sim, "true"),
                    OrSubstitution(
                        EqualsSubstitution(namespace, "coug2_dvldr"),
                        EqualsSubstitution(namespace, "coug2_ekf"),
                    ),
                )
            ),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "param_file",
                default_value="auv0",
                description="Parameter file for the AUV",
            ),
            DeclareLaunchArgument(
                "fleet_param",
                default_value="auv0",
                description="Fleet parameter file for the AUV",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
