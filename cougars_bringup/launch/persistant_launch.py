import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    Resolves launch arguments and constructs the child launch file actions.

    Called at launch time via OpaqueFunction, which allows LaunchConfiguration
    values to be resolved to real Python strings via .perform(context). This is
    necessary because the 'flags' argument must be parsed into individual
    key-value pairs before being forwarded to child launch files.

    All child launch files receive the same launch_args, which includes the
    base arguments (namespace, param_file, fleet_param, flags) plus any
    additional arguments parsed from the flags string.
    """

    ### Build launch_args with parsed flags
    launch_args = [
        ('namespace',   LaunchConfiguration('namespace').perform(context)),
        ('param_file',  LaunchConfiguration('param_file').perform(context)),
        ('fleet_param', LaunchConfiguration('fleet_param').perform(context)),
        ('flags',       LaunchConfiguration('flags').perform(context)),
    ]

    flags = parse_flags(LaunchConfiguration('flags').perform(context))
    launch_args.extend(flags.items())

    ### Package launch directories
    bridge_dir = os.path.join(
        get_package_share_directory('cougars_bridge'), 'launch')
    bringup_dir = os.path.join(
        get_package_share_directory('cougars_bringup'), 'launch')
    control_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')
    coms_dir = os.path.join(
        get_package_share_directory('cougars_coms'), 'launch')
    localization_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')

    ### Launch files
    return [
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_dir, "cougars_bridge_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coms_dir, "coms_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_dir, "control_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_dir, "waypoint_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_dir, "cougars_localization_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "sensors_launch.py")),
            launch_arguments=launch_args),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "recorder_diagnostics_launch.py")),
            launch_arguments=launch_args),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='coug0',
            description='Namespace for the vehicle'
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml',
            description='Path to the vehicle parameter file'
        ),
        DeclareLaunchArgument(
            'fleet_param',
            default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml',
            description='Path to the fleet parameter file'
        ),
        DeclareLaunchArgument(
            'flags',
            default_value='',
            description='Comma-separated list of launch argument overrides.'
        ),
        OpaqueFunction(function=launch_setup),
    ])


def parse_flags(flag_str: str) -> dict[str, str]:
    """
    Parses a comma-separated string of key:value pairs into a dict of strings.

    Example input: "use_gps:false,use_navigation:true,verbosity_level:5"

    Values are kept as strings, as they will be passed directly to child
    launch files as launch arguments. Bare keys (no value) are set to "true".
    """
    result = {}
    if not flag_str:
        return result

    for item in flag_str.split(","):
        item = item.strip()
        if not item:
            continue

        if ":" in item:
            k, v = item.split(":", 1)
            result[k.strip()] = v.strip()
        else:
            result[item] = "true"

    return result
