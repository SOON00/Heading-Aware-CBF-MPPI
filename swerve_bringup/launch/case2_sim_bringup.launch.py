import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction


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
            'world': 'case2',
            'model': 'openarm',
        }.items()
    )
    
    arm_initial_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/arm_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.717, -0.467, -0.773, 0.898, 0.637, 0.573, 0.110, -0.717, -0.467, 0.773, -0.898, -0.637, -0.573, -0.110]}',
            '--once'
        ],
        output='screen'
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
            'map': 'case2.yaml',
            'params': 'case2',
        }.items()
    )

    waypoint_sender = Node(
        package='swerve_navigation',
        executable='waypoint_goal_sender_case2',
        name='waypoint_goal_sender_case2',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,

        TimerAction(
            period=5.0,
            actions=[
                arm_initial_pose_pub,
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