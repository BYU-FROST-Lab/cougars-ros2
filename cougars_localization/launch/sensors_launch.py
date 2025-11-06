import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import yaml

def generate_launch_description():
    '''
    Launches the sensor nodes for the vehicle.
    '''

    param_file = '/home/frostlab/config/deploy_tmp/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/deploy_tmp/fleet_params.yaml'
    GPS = "False"  # Default to 'False'
    verbose = "False"
    namespace=''
    BLUEROV = "False"
 
    # Declare launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=namespace,
        description='Namespace for the vehicle'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=param_file,
        description='Path to the vehicle parameter file'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=fleet_param,          
        description='Path to the fleet parameter file'
    )

    launch_actions = []
    launch_actions.extend([
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
    ])

    # TODO if GPS is false
    dvl = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    launch_actions.append(dvl)

    # TODO if BLUEROV is false
    # Serial Teensy connection
    launch_actions.append(
        launch_ros.actions.Node(
        package='cougars_control', 
        executable='mc_serial_node', 
        namespace=LaunchConfiguration('namespace'),
        output='log',
    ))
    # TODO if BLUEROV is true
    # TODO DEFINETLY NEED TO FIX THIS
    # Pressure sensor for blueROV
    # launch_actions.append(launch_ros.actions.Node(
    #     package='pressure_sensor',
    #     executable='get_pressure',
    #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
    #     namespace=LaunchConfiguration('namespace'),
    #     output='log',
    # ))


    launch_actions.extend([
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='emergency_protocols',
        #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='dvl_manager.py',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
        ),


        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            output='log',
        ),

        # Setup the USBL modem
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            output='log',
        ),
        launch_ros.actions.Node(
            package='cougars_coms',
            executable='vehicle_pinger',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            output='log',
        ),
        launch_ros.actions.Node(
            package='cougars_coms',
            executable='rf_bridge.py',
            parameters=[LaunchConfiguration('param_file')],
            namespace=LaunchConfiguration('namespace'),
            output='log',
        ), 
        # Setup the GPS
        # TODO make sure these parameters are still being loaded correctly
        launch_ros.actions.ComposableNodeContainer(
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
        ),
    ])

    return launch.LaunchDescription(launch_actions)