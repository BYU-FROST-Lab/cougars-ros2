
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        'sim',
        default_value='False',
        description='Use simulation clock if true'
    )

    sim = LaunchConfiguration('sim')
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    fleet_param = LaunchConfiguration('fleet_param')

    # TODO should i put it in here or should i put it in the cougars_bringup?
    return LaunchDescription([
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_sim_time_launch_arg,

        # Node(
        #     package='cougars_localization',
        #     executable='raw_state_estimate.py',
        #     parameters=[param_file, fleet_param, {'use_sim_time': sim}],
        #     namespace=namespace,

        # ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            parameters=[param_file, fleet_param, {'use_sim_time': sim}],
            namespace=namespace,
            remappings=[('odometry/filtered', 'odometry/global')]    
        ),
    ])
