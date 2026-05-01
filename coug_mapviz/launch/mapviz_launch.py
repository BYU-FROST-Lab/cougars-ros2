from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def generate_launch_description():
    # base_path = os.getenv('HOME', '/home/frostlab/config')
    # Mapviz_config
    
    base_path = get_package_share_directory('coug_mapviz')
    config_path = os.path.join(base_path, 'mapviz', 'BlueROV.mvc')
    # Mapviz origins param file
    mapviz_origins_path = os.path.join(
        base_path, 'mapviz', 'mapviz_origins.yaml'
    )
    
    mapviz_origins = Path(mapviz_origins_path).read_text()
  
    return LaunchDescription([
        # Set environment variable
        # Declare launch arguments
        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('origin', default_value='default_site'),

        LogInfo(msg=['Config file path: ', config_path]),

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                'config': config_path,
            }],
        ),

        # Node for initialize_origin
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            # remappings=[('/fix', '/bluerov2/imu/nav_sat_fix')],
            parameters=[
                {'local_xy_frame': '/map'},
                {'local_xy_origin': LaunchConfiguration('origin')},
                {'local_xy_origins': mapviz_origins},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'origin'],
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "auto"},
                {"local_xy_navsatfix_topic": "/gps/fix"}
            ]
        ),

    ])
