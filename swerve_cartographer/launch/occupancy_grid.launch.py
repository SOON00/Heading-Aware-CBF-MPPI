from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation clock if true'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of each occupancy grid cell'
    )

    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Occupancy grid publish period in seconds'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        publish_period_sec_arg,
        occupancy_grid_node,
    ])