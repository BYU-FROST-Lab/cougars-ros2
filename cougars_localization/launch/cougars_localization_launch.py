
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from pathlib import Path

def generate_launch_description():

    # TODO: add localization
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
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    raw_state_estimate = launch_ros.actions.Node(
        package='cougars_localization',
        executable='raw_state_estimate.py',
        parameters=[LaunchConfiguration('param_file'), 
                    LaunchConfiguration('fleet_param'), 
                    {'use_sim_time': use_sim_time}],
        namespace=LaunchConfiguration('namespace')
    )

    return LaunchDescription([
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_sim_time_launch_arg,

        raw_state_estimate
    ])
