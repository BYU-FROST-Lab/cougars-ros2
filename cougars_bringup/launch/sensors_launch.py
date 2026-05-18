
import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from pathlib import Path

#######################################
# 
# Launches sensor nodes for vehicle
# 
# 
#######################################


def generate_launch_description():

    ### Declare launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace for the vehicle'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        description='Path to the vehicle parameter file'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',       
        description='Path to the fleet parameter file'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Use simulation clock if true'
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
    use_pressure_launch_arg = DeclareLaunchArgument(
        'use_pressure',
        default_value='true',
        description='Launch pressure sensor and depth converter nodes'
    )

    sim = LaunchConfiguration('sim')

    # Declare launch nodes
    dvl_node = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_sensor',
        parameters=[LaunchConfiguration('param_file'), 
                    LaunchConfiguration('fleet_param'), 
                    {'use_sim_time': sim}],
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('use_dvl')),
        output='log'
    )
    dvl_manager = launch_ros.actions.Node(
        package='cougars_bridge',
        executable='dvl_manager.py',
        parameters=[LaunchConfiguration('param_file'), 
                    LaunchConfiguration('fleet_param'), 
                    {'use_sim_time': sim}],
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('use_dvl')),
        output='log'
    )

    seatrac_node = launch_ros.actions.Node(
        package='seatrac',
        executable='modem',
        parameters=[LaunchConfiguration('param_file'), 
                    LaunchConfiguration('fleet_param'), 
                    {'use_sim_time': sim}],
        namespace=LaunchConfiguration('namespace'),
        output='log',
        condition=IfCondition(LaunchConfiguration('acoms_on')),
    )

    pressure_to_depth_node = launch_ros.actions.Node(
        package='pressure_sensor',
        executable='pressure_to_depth',
        parameters=[LaunchConfiguration('param_file'), 
                    LaunchConfiguration('fleet_param'), 
                    {'use_sim_time': sim}],
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('use_pressure')),
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
                    {'log_level': 'warn'},  # Add log level here
                    {'use_sim_time': sim}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            launch_ros.descriptions.ComposableNode(
                package='gps_tools',
                plugin='gps_tools::UtmOdometryComponent',
                namespace=LaunchConfiguration('namespace'),
                name='utm_gpsfix_to_odometry_node',
                parameters=[
                    {'log_level': 'warn'},  # Add log level here
                    {'use_sim_time': sim}
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
        sim_launch_arg,
        use_dvl_launch_arg,
        use_gps_launch_arg,
        use_acoustics_launch_arg,
        use_pressure_launch_arg,

        # launch nodes
        dvl_node,
        dvl_manager,
        seatrac_node,
        gps_node_container,
        pressure_to_depth_node
    ]

    return launch.LaunchDescription(launch_actions)
