# Launch file for the RF bridge node in the cougars_coms package

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

  ### Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', 
        default_value='coug0',
        description='Vehicle namespace (e.g., coug0)')
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/vehicle_params.yaml',
        description='Path to vehicle parameters YAML file')
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging')


  ### Define the node
    rf_bridge_node = Node(
        package='cougars_coms',
        executable='rf_bridge.py',
        name='rf_bridge',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('param_file'),
            {'debug_mode': LaunchConfiguration('debug')}
        ],
        output='screen',
        emulate_tty=True
    )

  ### Launch Actions
    launch_actions = [
        # launch arguments
        namespace_arg,
        param_file_arg,
        debug_arg,

        # launch node
        rf_bridge_node
    ]

    return LaunchDescription(launch_actions)