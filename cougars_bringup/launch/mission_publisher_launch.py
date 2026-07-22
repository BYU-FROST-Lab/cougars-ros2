import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from pathlib import Path


def generate_launch_description():
    ### Launch arguments
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    mission_file_launch_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=f'{Path.home()}/config/missions/mission.yaml',
        description='Path to the mission YAML file to load and publish.'
    )
    mission_key_launch_arg = DeclareLaunchArgument(
        'mission_key',
        default_value='',
        description='If set, only publish this one key from the mission file instead of all of them.'
    )
    topic_launch_arg = DeclareLaunchArgument(
        'topic',
        default_value='',
        description='If set, forces every published entry onto this topic instead of each entry\'s own "topic" field.'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Use simulation clock if true'
    )

    ### Launch Nodes
    mission_publisher = launch_ros.actions.Node(
        package='cougars_bringup',
        executable='mission_publisher.py',
        name='mission_publisher',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'mission_key': LaunchConfiguration('mission_key'),
            'topic': LaunchConfiguration('topic'),
            'use_sim_time': LaunchConfiguration('sim'),
        }],
        namespace=LaunchConfiguration('namespace'),
        output='screen',
    )

    launch_actions = [
        # launch arguments
        namespace_launch_arg,
        mission_file_launch_arg,
        mission_key_launch_arg,
        topic_launch_arg,
        sim_launch_arg,

        # launch nodes
        mission_publisher,
    ]

    return launch.LaunchDescription(launch_actions)
