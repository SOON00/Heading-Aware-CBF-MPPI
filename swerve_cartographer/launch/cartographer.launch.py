import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cartographer_pkg = get_package_share_directory('swerve_cartographer')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=['sim', 'real'],
        description='Select execution mode: sim or real'
    )

    mode = LaunchConfiguration('mode')

    use_sim_time = PythonExpression([
        "'true' if '", mode, "' == 'sim' else 'false'"
    ])

    cartographer_config_dir = os.path.join(
        cartographer_pkg,
        'config'
    )

    configuration_basename = 'swerve_2d.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ]
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                cartographer_pkg,
                'launch',
                'occupancy_grid.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz_config = os.path.join(
        cartographer_pkg,
        'rviz',
        'cartographer.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        mode_arg,
        cartographer_node,
        occupancy_grid,
        rviz,
    ])