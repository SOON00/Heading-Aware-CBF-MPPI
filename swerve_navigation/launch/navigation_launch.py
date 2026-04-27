import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _get_available_maps(maps_dir):
    if not os.path.isdir(maps_dir):
        return []

    return sorted([
        filename
        for filename in os.listdir(maps_dir)
        if filename.endswith('.yaml')
    ])


def _format_available_maps(valid_maps):
    if not valid_maps:
        return '  No .yaml maps found'

    return '\n'.join([
        f'  - {map_name}'
        for map_name in valid_maps
    ])


def _get_available_params(params_dir, mode_value):
    if not os.path.isdir(params_dir):
        return []

    prefix = f'nav2_params_{mode_value}_'

    return sorted([
        filename.replace(prefix, '').replace('.yaml', '')
        for filename in os.listdir(params_dir)
        if filename.startswith(prefix) and filename.endswith('.yaml')
    ])


def _format_available_params(valid_params):
    if not valid_params:
        return '  No matching params files found'

    return '\n'.join([
        f'  - {params_name}'
        for params_name in valid_params
    ])


def _launch_navigation(context, *args, **kwargs):
    navigation_pkg = get_package_share_directory('swerve_navigation')
    cartographer_pkg = get_package_share_directory('swerve_cartographer')

    mode_value = LaunchConfiguration('mode').perform(context)
    map_value = LaunchConfiguration('map').perform(context)
    params_value = LaunchConfiguration('params').perform(context)

    maps_dir = os.path.join(
        cartographer_pkg,
        'maps'
    )

    valid_maps = _get_available_maps(maps_dir)

    if os.path.isabs(map_value) or os.path.dirname(map_value):
        raise ValueError(
            "\nInvalid map argument.\n"
            "Please provide only a map file name from swerve_cartographer/maps.\n\n"
            "Example:\n"
            "  map:=my_map.yaml\n\n"
            "Available maps:\n"
            f"{_format_available_maps(valid_maps)}\n"
        )

    if not map_value.endswith('.yaml'):
        raise ValueError(
            f"\nInvalid map name: '{map_value}'\n"
            "The map argument must include the .yaml extension.\n\n"
            "Example:\n"
            "  map:=my_map.yaml\n\n"
            "Available maps:\n"
            f"{_format_available_maps(valid_maps)}\n"
        )

    map_yaml_file = os.path.join(
        maps_dir,
        map_value
    )

    if not os.path.isfile(map_yaml_file):
        raise FileNotFoundError(
            f"\nMap file '{map_value}' was not found.\n"
            "Maps must be placed in swerve_cartographer/maps.\n\n"
            "Looked for:\n"
            f"  {map_yaml_file}\n\n"
            "Available maps:\n"
            f"{_format_available_maps(valid_maps)}\n"
        )

    use_sim_time_bool = mode_value == 'sim'
    use_sim_time = 'true' if use_sim_time_bool else 'false'

    params_dir = os.path.join(
        navigation_pkg,
        'params'
    )

    valid_params = _get_available_params(
        params_dir,
        mode_value
    )

    if os.path.isabs(params_value) or os.path.dirname(params_value):
        raise ValueError(
            "\nInvalid params argument.\n"
            "Please provide only a params profile name.\n\n"
            "Example:\n"
            "  params:=case1\n\n"
            f"Available params for mode '{mode_value}':\n"
            f"{_format_available_params(valid_params)}\n"
        )

    params_filename = f'nav2_params_{mode_value}_{params_value}.yaml'

    params_file = os.path.join(
        params_dir,
        params_filename
    )

    if not os.path.isfile(params_file):
        raise FileNotFoundError(
            f"\nParams profile '{params_value}' was not found for mode '{mode_value}'.\n"
            "Params files must be placed in swerve_navigation/params.\n\n"
            "Expected file:\n"
            f"  {params_file}\n\n"
            f"Available params for mode '{mode_value}':\n"
            f"{_format_available_params(valid_params)}\n"
        )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'log_level': 'info'
        }.items()
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': 'true'
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

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings + [
            ('cmd_vel', 'cmd_vel_nav')
        ]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            configured_params
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        remappings=remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_bool,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ]
    )

    print(f"[swerve_navigation] mode: {mode_value}")
    print(f"[swerve_navigation] use_sim_time: {use_sim_time}")
    print(f"[swerve_navigation] map: {map_yaml_file}")
    print(f"[swerve_navigation] params: {params_file}")

    return [
        localization_launch,

        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
    ]


def generate_launch_description():
    navigation_pkg = get_package_share_directory('swerve_navigation')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=[
            'sim',
            'real'
        ],
        description='Select execution mode: sim or real'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        description='Map yaml file name in swerve_cartographer/maps, for example my_map.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params',
        description='Nav2 params profile name, for example case1'
    )

    navigation = OpaqueFunction(
        function=_launch_navigation
    )

    rviz_config = os.path.join(
        navigation_pkg,
        'rviz',
        'navigation.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config
        ],
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        params_arg,
        navigation,
        rviz,
    ])