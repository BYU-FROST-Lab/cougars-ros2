
import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

#######################################
# 
# Launches sensor nodes for vehicle
# 
# 
#######################################


def generate_launch_description():

    # Declare launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the vehicle'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/agent/vehicle_params.yaml',
        description='Path to the vehicle parameter file'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/fleet/fleet_params.yaml',          
        description='Path to the fleet parameter file'
    )

    use_dvl_launch_arg = DeclareLaunchArgument(
        'use_dvl',
        default_value='true',
        description='Launch DVL sensor and manager nodes'
    )
    use_gps_launch_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='Launch GPS nodes'
    )
    use_acoustics_launch_arg = DeclareLaunchArgument(
        'acoms_on',
        default_value='true',
        description='Launch Seatrac acoustic modem node'
    )

    # Declare launch nodes
    dvl_node = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_sensor',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('use_dvl')),
    )
    dvl_manager = launch_ros.actions.Node(
        package='cougars_bridge',
        executable='dvl_manager.py',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('use_dvl')),
    )

    seatrac_node = launch_ros.actions.Node(
        package='seatrac',
        executable='modem',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        output='log',
        condition=IfCondition(LaunchConfiguration('acoms_on')),
    )

    gps_node_container = launch_ros.actions.ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='fix_and_odometry_container',
        namespace=LaunchConfiguration('namespace'),
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='gpsd_client',
                plugin='gpsd_client::GPSDClientComponent',
                # name='gpsd_client',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    LaunchConfiguration('param_file'),
                    LaunchConfiguration('fleet_param'),
                    # vehicle_params[namespace]['gpsd_client']['ros__parameters'],
                    {'log_level': 'warn'}  # Add log level here
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            launch_ros.descriptions.ComposableNode(
                package='gps_tools',
                plugin='gps_tools::UtmOdometryComponent',
                namespace=LaunchConfiguration('namespace'),
                name='utm_gpsfix_to_odometry_node',
                parameters=[
                    {'log_level': 'warn'}  # Add log level here
                ],
            ),
        ],
        output='log',
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('use_gps')),
    )

    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_dvl_launch_arg,
        use_gps_launch_arg,
        use_acoustics_launch_arg,

        # launch nodes
        dvl_node,
        dvl_manager,
        seatrac_node,
        gps_node_container
    ]

    return launch.LaunchDescription(launch_actions)
