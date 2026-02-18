
import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
        default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml',
        description='Path to the vehicle parameter file'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml',          
        description='Path to the fleet parameter file'
    )

    # Declare launch nodes
    dvl_node = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    dvl_manager = launch_ros.actions.Node(
        package='cougars_bridge',
        executable='dvl_manager.py',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )

    seatrac_node = launch_ros.actions.Node(
        package='seatrac',
        executable='modem',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        output='log',
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
    )

    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,

        # launch nodes
        dvl_node,
        dvl_manager,
        seatrac_node,
        gps_node_container
    ]

    return launch.LaunchDescription(launch_actions)
