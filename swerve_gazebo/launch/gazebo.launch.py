import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _find_world_path(context, *args, **kwargs):
    world_value = LaunchConfiguration('world').perform(context)

    if os.path.isfile(world_value):
        world_file = world_value
    else:
        gazebo_pkg = get_package_share_directory('swerve_gazebo')
        worlds_dir = os.path.join(gazebo_pkg, 'worlds')

        if world_value.endswith('.world'):
            world_file = os.path.join(worlds_dir, world_value)
        else:
            world_file = os.path.join(worlds_dir, world_value + '.world')

        if not os.path.isfile(world_file):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_file}"
            )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return [gzserver, gzclient]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='swerve',
        choices=['swerve', 'openarm'],
        description='Select robot model: swerve or openarm'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name in swerve_gazebo/worlds, .world file name, or full path'
    )

    model = LaunchConfiguration('model')
    
    xacro_filename = PythonExpression([
        "'swerve_openarm_robot_sim.urdf.xacro' if '", model,
        "' == 'openarm' else 'swerve_robot_sim.urdf.xacro'"
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('swerve_gazebo'),
        'urdf',
        xacro_filename
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    controller_yaml_filename = PythonExpression([
        "'controller_swerve_openarm.yaml' if '", model,
        "' == 'openarm' else 'controller_swerve.yaml'"
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('swerve_gazebo'),
        'config',
        controller_yaml_filename
    ])

    gazebo_pkg = get_package_share_directory('swerve_gazebo')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_models_path = os.path.join(gazebo_pkg, 'models')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            gazebo_models_path,
            os.pathsep,
            existing_model_path
        ] if existing_model_path else gazebo_models_path
    )
    
    gazebo = OpaqueFunction(function=_find_world_path)

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
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'swerve_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', robot_controllers
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    swerve_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', robot_controllers
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    swerve_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', robot_controllers
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', robot_controllers
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        condition=IfCondition(PythonExpression([
            "'", model, "' == 'openarm'"
        ])),
        output='screen'
    )

    swerve_drive_controller = Node(
        package='swerve_controller',
        executable='swerve_drive_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    odom_publisher_node = Node(
        package='swerve_controller',
        executable='swerve_odom_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )

    steering_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                swerve_steering_controller_spawner
            ]
        )
    )

    velocity_controller_after_steering = RegisterEventHandler(
        OnProcessExit(
            target_action=swerve_steering_controller_spawner,
            on_exit=[
                swerve_velocity_controller_spawner
            ]
        )
    )

    arm_controller_after_velocity = RegisterEventHandler(
        OnProcessExit(
            target_action=swerve_velocity_controller_spawner,
            on_exit=[
                arm_position_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_after_spawn,
        steering_controller_after_joint_state,
        velocity_controller_after_steering,
        arm_controller_after_velocity,
        swerve_drive_controller,
        odom_publisher_node,
    ])