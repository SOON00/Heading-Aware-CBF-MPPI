import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='swerve',
        choices=['swerve', 'openarm'],
        description='Select robot model: swerve or openarm'
    )

    model = LaunchConfiguration('model')

    xacro_filename = PythonExpression([
        "'swerve_openarm_robot.urdf.xacro' if '", model,
        "' == 'openarm' else 'swerve_robot.urdf.xacro'"
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('swerve_description'),
        'urdf',
        xacro_filename
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            robot_description
        ]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('swerve_description'),
        'rviz',
        'description.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])