import launch
import launch_ros.actions
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression

import os
from pathlib import Path

#########################################
# 
# Launches all the converter nodes that bridge
# the cougars hardward nodes with the algorithmic nodes
# 
#########################################


def generate_launch_description():

    ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=f'{Path.home()}/config/agent/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=f'{Path.home()}/config/fleet/fleet_params.yaml'
    )
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Use simulation clock if true'
    )

    sim = LaunchConfiguration('sim')
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    fleet_param = LaunchConfiguration('fleet_param')

    dvl_odom_frame_id = PythonExpression(
        [
            "'",
            namespace,
            "/dvl_odom' if '",
            namespace,
            "' != '' else 'dvl_odom'",
        ]
    )

    dvl_odom_ned_frame_id = PythonExpression(
        [
            "'",
            namespace,
            "/dvl_odom_ned' if '",
            namespace,
            "' != '' else 'dvl_odom_ned'",
        ]
    )

    
    # TODO fix this as well. Launching from another package
    # declare nodes
    depth_converter = Node(
        package='pressure_sensor',
        executable='pressure_to_depth',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
    )
    # dvl_converter = Node(
    #     package='cougars_bridge',
    #     executable='dvl_converter',
    #     parameters=[param_file, fleet_param, {'use_sim_time': sim}],
    #     namespace=namespace,
    # )
    # TODO launching a node from another package but not sure where to put it.
    dvl_converter = Node(
        package='dvl_a50',
        executable='dvl_a50_nav',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
    )
    transform_publisher = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_zup_to_zdown',
            namespace=namespace,
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0', 
                '--qx', '1', 
                '--qy', '0', 
                '--qz', '0', 
                '--qw', '0',
                '--frame-id', dvl_odom_frame_id,
                '--child-frame-id', dvl_odom_ned_frame_id],
            output='screen',
        )

    # dvl_global = Node(
    #     package='cougars_bridge',
    #     executable='dvl_global',
    #     parameters=[param_file, fleet_param, {'use_sim_time': sim}],
    #     namespace=namespace,
    # )
    seatrac_ahrs_convertor = Node(
        package='cougars_bridge',
        executable='seatrac_imu_converter',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
    )

    gps_odom = Node(
        package='cougars_bridge',
        executable='gps_odom.py',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
        remappings=[('fix', 'imu/nav_sat_fix')]
    )

    imu_source_vehicle = Node(
        package='topic_tools',
        name='vehicle_imu_source',
        executable='relay',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    static_tf_publisher = Node(
        package='cougars_bridge',
        executable='static_tf_publisher',
        parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        namespace=namespace,
    )


    # compile launch actions
    launch_actions = [

        # launch arguements
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_sim_time_launch_arg,

        # launch nodes
        depth_converter,
        dvl_converter,
        # dvl_global,
        transform_publisher,
        seatrac_ahrs_convertor,
        gps_odom,
        static_tf_publisher,
        imu_source_vehicle
    ]

    return launch.LaunchDescription(launch_actions)