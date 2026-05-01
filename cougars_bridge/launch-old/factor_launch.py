import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024

    Launches the MOOS-IvP control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    config_file = "/home/frostlab/config/vehicle_config.yaml"

    # Get the directory of the launch files
    # package_dir = os.path.join(
    #     get_package_share_directory('cougars_control'), 'launch')

    
    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Unique vehicle namespace'
        ),
        # # Include additional launch files
        # launch.actions.IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(package_dir, 'sensors_launch.py'))
        # ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[config_file],
            namespace=LaunchConfiguration('namespace'),
        ),
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='coug_controls',
        #     parameters=[config_file],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='moos_bridge',
        #     parameters=[config_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output='screen',
        # ),
        # Start the EmergencyStop checks
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='leak_sub.py',
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='battery_sub.py',
        #     parameters=[config_file],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
    ])
