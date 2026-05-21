import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description():

    ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=os.getenv('NAMESPACE', 'coug0')
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=f'{Path.home()}/config/agent/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=f'{Path.home()}/config/fleet/fleet_params.yaml'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )

    launch_args = [
        ('namespace',   LaunchConfiguration('namespace')),
        ('param_file',  LaunchConfiguration('param_file')),
        ('fleet_param', LaunchConfiguration('fleet_param')),
        ('use_sim_time', LaunchConfiguration('sim'))
    ]



    ### Package launch directories
    bridge_dir = os.path.join(
        get_package_share_directory('cougars_bridge'), 'launch')
    bringup_dir = os.path.join(
        get_package_share_directory('cougars_bringup'), 'launch')
    control_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')
    coms_dir = os.path.join(
        get_package_share_directory('cougars_coms'), 'launch')
    localization_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')
    nav_dir = os.path.join(
        get_package_share_directory('cougars_nav'), 'launch')
    description_dir = os.path.join(
        get_package_share_directory('coug_description'), 'launch')
    description_urdf_dir = os.path.join(
        get_package_share_directory('coug_description'), 'urdf')

    try:    
        holoocean_bridge_dir = os.path.join(
            get_package_share_directory('sim_converters'), 'launch')
    except:
        holoocean_bridge_dir = None


    ### launch files
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_dir, "cougars_bridge_launch.py")),
        launch_arguments=launch_args)

    coms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coms_dir, "coms_launch.py")),
        launch_arguments=launch_args)

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, "control_launch.py")),
        launch_arguments=launch_args)

    waypoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, "waypoint_launch.py")),
        launch_arguments=launch_args)

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, "cougars_localization_launch.py")),
        launch_arguments=launch_args)

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "bringup_launch.py")),
        launch_arguments=launch_args)

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "sensors_launch.py")),
        launch_arguments=launch_args,
        condition=UnlessCondition(LaunchConfiguration('sim')))

    sim_coug_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, "coug_description.launch.py")),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('sim', LaunchConfiguration('sim')),
            ('urdf_file', os.path.join(description_urdf_dir, 'couguv_holoocean.urdf.xacro')),
        ],
        condition=IfCondition(LaunchConfiguration('sim')))

    coug_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, "coug_description.launch.py")),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('sim', LaunchConfiguration('sim')),
            ('urdf_file', os.path.join(description_urdf_dir, 'couguv.urdf.xacro')),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim')))

    if holoocean_bridge_dir is not None:
        holo_bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(holoocean_bridge_dir, "reverse_launch.py")),
            launch_arguments=launch_args,
            condition=IfCondition(LaunchConfiguration('sim')))



    launch_actions = [
        # launch args
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        sim_launch_arg,

        # launch files
        bridge_launch,
        coms_launch,
        control_launch,
        waypoint_launch,
        localization_launch,
        sensors_launch,
        coug_description_launch,
        sim_coug_description_launch,
        bringup_launch,
    ]

    if holoocean_bridge_dir is not None:
        launch_actions.extend([
            holo_bridge_launch
        ])

    return launch.LaunchDescription(launch_actions)

