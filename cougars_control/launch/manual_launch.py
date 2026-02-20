import launch
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def debug_launch_args(context, *args, **kwargs):
    demo_val = LaunchConfiguration('demo').perform(context)
    sim_val = LaunchConfiguration('sim').perform(context)
    print(f"[DEBUG] demo = {demo_val}, sim = {sim_val}")
    return []

def generate_launch_description():
    '''
    Launches the manual control and sensor nodes for the vehicle.
    '''

    # default parameter file paths
    param_file = '/home/frostlab/config/deploy_tmp/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/deploy_tmp/fleet_params.yaml'
    # Get the directory of the launch files
    control_package_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')

    # launch parameters
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    # if sim is true, 
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )
    demo_launch_arg = DeclareLaunchArgument(
        'demo',
        default_value='False'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=param_file
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=fleet_param
    )
    # TODO fix verbose mode
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False',    
    )
    
    launch_actions = []
    launch_actions.extend([
        namespace_launch_arg,
        sim_launch_arg,
        demo_launch_arg,
        param_file_launch_arg,
        fleet_param_launch_arg,
        verbose_launch_arg,
    ])

    # Include demo if sim == True AND demo == True
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_package_dir, "demo_launch.py")
        ),
        condition=IfCondition(
            PythonExpression([
                LaunchConfiguration('sim'), ' and ',
                LaunchConfiguration('demo')
            ])
        )
    )


    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_package_dir, "control_launch.py")
        ),
        condition=IfCondition(
            PythonExpression([
                LaunchConfiguration('sim'), ' and ',
                LaunchConfiguration('demo')
            ])
        )
    )

    launch_actions.append(OpaqueFunction(function=debug_launch_args))
    launch_actions.append(sensors)
    launch_actions.append(demo)

    # Fins manual node — only if fins is true
    fins_manual = Node(
        package='cougars_control',
        executable='fins_manual.py',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        output='log',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('fins'))
    )

    # Manual mission — only if fins is false
    manual_mission = Node(
        package='cougars_control',
        executable='manual_mission.py',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        output='log',
        condition=UnlessCondition(LaunchConfiguration('fins'))
    )

    # Controls node — also only if fins is false
    controls = Node(
        package='cougars_control',
        executable='coug_controls',
        parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
        namespace=LaunchConfiguration('namespace'),
        output='log',
        condition=UnlessCondition(LaunchConfiguration('fins'))
    )

    # Add all to launch_actions (launch system will evaluate conditions at runtime)
    launch_actions.append(fins_manual)
    launch_actions.append(manual_mission)
    launch_actions.append(controls)
    
    
    launch_actions.extend([
        
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_package_dir, "converters_launch.py"))
        ),  
        #start factor graph for logging
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[LaunchConfiguration('param_file')],
            namespace=LaunchConfiguration('namespace'),
            output='log',
        ), 


        # Start the rf_bridge node
        # launch_ros.actions.Node(
        #     package='cougars_coms',
        #     executable='rf_bridge.py',
        #     parameters=[LaunchConfiguration('param_file')],
        #     namespace=LaunchConfiguration('namespace'),
        #     output='log',
        # ), 
        
    ])

    return launch.LaunchDescription(launch_actions)