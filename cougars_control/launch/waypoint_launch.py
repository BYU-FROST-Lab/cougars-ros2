import sys
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    '''
    Launches the waypoint following mission stack.
    Accepts a 'mission_file' argument to specify which mission to run.
    Accepts a 'GPS' argument to choose between waypoint follower types.
    '''

    # --- Parameter Parsing ---
    sim = "false"
    verbose = "false"
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = 'coug4'
    mission_file = '/home/frostlab/config/test_mission.yaml' # Default mission file
    GPS = "false"  # Initialize GPS to false (default)

    # --- Diagnostic Print for sys.argv ---
    print(f"[INFO] [launch] Raw sys.argv: {sys.argv}")

    for arg_idx, arg_val in enumerate(sys.argv):
        print(f"[INFO] [launch] sys.argv[{arg_idx}]: '{arg_val}'") # Print each argument
        if arg_val.startswith('namespace:='):
            namespace = arg_val.split(':=', 1)[1]
        elif arg_val.startswith('param_file:='):
            param_file = arg_val.split(':=', 1)[1]
        elif arg_val.startswith("sim:="):
            sim = arg_val.split(":=", 1)[1].lower()
        elif arg_val.startswith("verbose:="):
            verbose = arg_val.split(":=", 1)[1].lower()
        elif arg_val.startswith("GPS:="): # Parse GPS flag from command line
            parsed_gps_value = arg_val.split(":=", 1)[1].lower()
            print(f"[INFO] [launch] Found GPS argument: '{arg_val}', Parsed value: '{parsed_gps_value}'")
            GPS = parsed_gps_value
        elif arg_val.startswith("mission_file:="):
            mission_file = arg_val.split(":=", 1)[1]

    # --- Diagnostic Print for GPS flag ---
    print(f"[INFO] [launch] Final Parsed GPS flag: '{GPS}' (Type: {type(GPS)})")
    print(f"[INFO] [launch] Condition GPS == 'true' is: {GPS == 'true'}")

    output_config = 'screen' if verbose == "true" else 'log'

    # --- Package Directories ---
    localization_pkg_dir = get_package_share_directory('cougars_localization')
    # config_pkg_dir = get_package_share_directory('config')

    # --- Mission File Path ---
    mission_yaml_path = mission_file
    print(f"[INFO] [launch] Using mission file: {mission_yaml_path}")

    # --- Launch Actions List (Initialize) ---
    launch_actions = []

    # --- Include Other Launch Files First ---
    if sim == "false":
        sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_pkg_dir, 'launch', "sensors_launch.py")
            )
        )
        launch_actions.append(sensors_launch)

    converters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg_dir, 'launch', "converters_launch.py")
        )
    )
    launch_actions.append(converters_launch)

    # --- Define the Waypoint Follower Node (conditionally) ---
    waypoint_node_to_launch: Node 

    if GPS == "true":
        print("[INFO] [launch] USING GPS FLAG: Launching waypoint_follower_gps")
        waypoint_node_to_launch = Node(
            package='cougars_control',
            executable='waypoint_follower_gps',
            name='waypoint_follower_gps_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {'waypoint_file_path': mission_yaml_path},
                {'slip_radius': 3.0},
                {'desired_travel_speed': 25.0},
                {'loop_rate': 5.0}
            ]
        )
    else:
        print("[INFO] [launch] NOT USING GPS FLAG: Launching waypoint_follower")
        waypoint_node_to_launch = Node(
            package='cougars_control',
            executable='waypoint_follower',
            name='waypoint_follower_node',
            namespace=namespace,
            output='screen',
            parameters=[param_file]
            
        )

    # --- Extend launch_actions with all defined Node objects ONCE ---
    launch_actions.extend([
        waypoint_node_to_launch,
    ])

    # --- Return Launch Description ---
    return LaunchDescription(launch_actions)