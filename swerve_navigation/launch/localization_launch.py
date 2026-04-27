import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_pkg = get_package_share_directory('swerve_navigation')

    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            navigation_pkg,
            'params',
            'nav2_params.yaml'
        ),
        description='Full path to Nav2 parameters file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically start lifecycle nodes'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level'
    )

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'map_server',
        'amcl'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ],
        remappings=remappings
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ],
        remappings=remappings
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        log_level_arg,

        map_server,
        amcl,
        lifecycle_manager,
    ])