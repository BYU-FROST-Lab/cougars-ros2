
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch_ros.descriptions


def generate_launch_description():
    '''
    Launches the sensor nodes for the vehicle.
    '''
    param_file = '/home/frostlab/config/agent/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/fleet/fleet_params.yaml'
    GPS = "false"  # Default to 'false'
    namespace = 'coug0'

    # Declare launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=namespace,
        description='Namespace for the vehicle'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
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

    # TODO figure out how to do this better. Don't want to hardcode the path. 
    # I wont have the sim converters on the vehicle. It still might try and resolve the path. 
    # I could put the sim converters launch file in a package that will be on the vehicle
    sim_converters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/frostlab/ros2_ws/install/sim_converters/share/sim_converters/launch/reverse_launch.py'),
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    launch_actions = []
    launch_actions.extend([
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        sim_launch_arg,
        sim_converters,
    ])
    depth = launch_ros.actions.Node(
        package='pressure_sensor',
        executable='pressure_to_depth',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    dvl = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_nav',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
    )
    # dvl_global = launch_ros.actions.Node(
    #     package='cougars_localization',
    #     executable='dvl_global',
    #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
    #     namespace=LaunchConfiguration('namespace'),
    # )


    launch_actions.extend([
        depth, 
        dvl, 
        # dvl_global
    ])

    launch_actions.extend([
        # Start the data conversion nodes
        # launch_ros.actions.Node(
        #     package='cougars_coms',
        #     executable='cougars_coms',
        #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='seatrac_ahrs_convertor',
        #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        # TODO need to rework
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='gps_odom.py',
        #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        #     namespace=LaunchConfiguration('namespace'),
        # ),

        # Replace with URDF static transform publisher
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='static_tf_publisher',
        #     parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        #     namespace=LaunchConfiguration('namespace')
        # )
    ])

    return launch.LaunchDescription(launch_actions)