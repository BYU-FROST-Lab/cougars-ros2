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
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs) -> list:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_tuples = yaml.safe_load(agent_list_str)
    agent_namespaces = [t[0] for t in agent_tuples]
    is_multiagent = len(agent_namespaces) > 1

    pkg_share = get_package_share_directory("coug_mapviz")
    full_template_path = os.path.join(pkg_share, "mapviz", "mapviz_config.mvc.template")

    if is_multiagent:
        with open(full_template_path, "r") as f:
            base = yaml.safe_load(f.read())

        global_display_types = {"mapviz_plugins/tile_map", "coug_mapviz/coug_waypoints"}
        base["displays"] = [
            d for d in base["displays"] if d["type"] in global_display_types
        ]

        per_agent_template_path = os.path.join(
            pkg_share, "mapviz", "multi_mapviz_config.mvc.template"
        )
        with open(per_agent_template_path, "r") as f:
            per_agent_template = f.read()

        for ns in agent_namespaces:
            per_agent = yaml.safe_load(per_agent_template.replace("AUV_NS", ns))
            base["displays"].extend(per_agent["displays"])

        temp_config = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".mvc")
        yaml.safe_dump(base, temp_config)
        temp_config.close()
    else:
        with open(full_template_path, "r") as f:
            config_content = f.read().replace("AUV_NS", agent_namespaces[0])

        temp_config = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".mvc")
        temp_config.write(config_content)
        temp_config.close()

    mapviz_config_file = temp_config.name

    return [
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[
                {"config": mapviz_config_file},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "/origin"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_origin_transform",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--yaw",
                "0",
                "--pitch",
                "0",
                "--roll",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "origin",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[[auv0, bluerov2.urdf.xacro]]",
                description=(
                    "YAML list of [auv_ns, auv_urdf] pairs "
                    "(e.g. '[[coug0sim, couguv_holoocean.urdf.xacro], "
                    "[coug1sim, couguv_holoocean.urdf.xacro]]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
