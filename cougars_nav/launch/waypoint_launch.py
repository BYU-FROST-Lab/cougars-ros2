import launch
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from pathlib import Path

def generate_launch_description():
    '''
    Launches the waypoint navigation nodes for the vehicle.
    '''

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
    

    ### Nodes
    waypoint_iterator = Node(
        package='cougars_nav',
        executable='waypoint_iterator',
        name='waypoint_iterator',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('param_file'), 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    waypoint_controller = Node(
        package='cougars_nav',
        executable='waypoint_controller',
        name='waypoint_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('param_file'), 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('state_estimate', '/holoocean/auv0/DynamicsSensorOdom')],
        output='screen',
    )
    setpoint_transformer_node = Node(
        package='cougars_nav',
        executable='setpoint_transformer.py',
        name='setpoint_transformer',
        namespace=LaunchConfiguration('namespace'),
        output='log',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )



    launch_actions = [
        # launch args
        namespace_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        use_sim_time_launch_arg,

        # nodes
        waypoint_iterator,
        waypoint_controller,
        setpoint_transformer_node
    ]

    return launch.LaunchDescription(launch_actions)
