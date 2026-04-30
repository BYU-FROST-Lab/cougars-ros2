import sys

import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''

    Launches the MOOS-IvP control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("sim:="):
            sim = arg.split(":=")[1].lower()
        if arg.startswith("GPS:="):
            GPS = arg.split(":=")[1].lower()
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')

    
    launch_actions = []

    if sim == "false":
        sensors = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "sensors_launch.py"))
        )
        launch_actions.append(sensors)


    moos = launch_ros.actions.Node(
        package='cougars_control',
        executable='moos_bridge',
        parameters=[param_file,
                    {'gps': GPS},
                    {'sim':sim}],
        namespace=namespace,
        output=output,
    )
    launch_actions.append(moos)
        


    launch_actions.extend([

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "converters_launch.py"))
        ),   
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem_pinger',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='cougars_coms',
            executable='rf_bridge.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        # Start the EmergencyStop checks
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='leak_sub.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='battery_sub.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        # ),
    ])

    return launch.LaunchDescription(launch_actions)