import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    swerve_gazebo_dir = get_package_share_directory('swerve_gazebo')
    swerve_navigation_dir = get_package_share_directory('swerve_navigation')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                swerve_gazebo_dir,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': 'case1',
            'model': 'swerve',
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                swerve_navigation_dir,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'mode': 'sim',
            'map': 'case1.yaml',
            'params': 'case1',
        }.items()
    )

    waypoint_sender = Node(
        package='swerve_navigation',
        executable='waypoint_goal_sender_case1',
        name='waypoint_goal_sender_case1',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,

        TimerAction(
            period=5.0,
            actions=[
                navigation_launch
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                waypoint_sender
            ]
        ),
    ])