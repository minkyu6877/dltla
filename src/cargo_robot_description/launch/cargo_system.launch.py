import atexit
import os
import shutil
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('cargo_robot_description')

    robot1_urdf_path = os.path.join(
        pkg_share, 'urdf', 'cargo_robot_1.urdf'
    )

    robot2_urdf_path = os.path.join(
        pkg_share, 'urdf', 'cargo_robot_2.urdf'
    )

    controller_config = os.path.join(
        pkg_share, 'config', 'mecanum_drive_controller.yaml'
    )

    # Work around a Jazzy gz_ros2_control bug that drops nested parameter
    # file paths when forwarding controller node arguments.
    temporary_config = tempfile.NamedTemporaryFile(
        prefix='cargo_robot_controller_', suffix='.yaml', delete=False)
    gazebo_controller_config = temporary_config.name
    temporary_config.close()
    shutil.copyfile(controller_config, gazebo_controller_config)
    atexit.register(
        lambda: os.path.exists(gazebo_controller_config)
        and os.remove(gazebo_controller_config)
    )

    with open(robot1_urdf_path, 'r') as f:
        robot1_description = f.read().replace(
            '__CONTROLLER_CONFIG__', gazebo_controller_config)

    with open(robot2_urdf_path, 'r') as f:
        robot2_description = f.read().replace(
            '__CONTROLLER_CONFIG__', gazebo_controller_config)

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_share, 'worlds', 'warehouse_l_shape.sdf')],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'
        }
    )

    # 2. Gazebo clock -> ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # 3. Robot State Publisher
    robot1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[
            {
                'robot_description': robot1_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    robot2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[
            {
                'robot_description': robot2_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # 4. Robot spawn
    spawn_robot1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'warehouse_l_shape',
            '-string', robot1_description,
            '-name', 'cargo_robot_1',
            '-x', '1.85',
            '-y', '0.30',
            '-z', '0.02'
        ],
        output='screen'
    )

    spawn_robot2 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'warehouse_l_shape',
            '-string', robot2_description,
            '-name', 'cargo_robot_2',
            '-x', '2.15',
            '-y', '0.30',
            '-z', '0.02'
        ],
        output='screen'
    )

    # 5. Controllers
    robot1_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/robot1/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    robot1_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '-c', '/robot1/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    robot2_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/robot2/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    robot2_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '-c', '/robot2/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    # 6. Mission Manager
    mission_manager = Node(
        package='cargo_fleet_manager',
        executable='mission_manager',
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 7. UWB Simulator
    uwb_simulator = Node(
        package='cargo_fleet_manager',
        executable='uwb_simulator',
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 8. QR Reader
    qr_reader = Node(
        package='cargo_fleet_manager',
        executable='qr_reader',
        condition=IfCondition(LaunchConfiguration('start_qr_reader')),
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_qr_reader',
            default_value='false',
            description='Start the physical QR camera reader.'),

        gazebo,

        TimerAction(
            period=1.0,
            actions=[
                clock_bridge,
                robot1_rsp,
                robot2_rsp
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[spawn_robot1]
        ),

        TimerAction(
            period=4.0,
            actions=[spawn_robot2]
        ),

        TimerAction(
            period=8.0,
            actions=[
                robot1_jsb,
                robot2_jsb
            ]
        ),

        TimerAction(
            period=9.0,
            actions=[
                robot1_mecanum,
                robot2_mecanum
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[uwb_simulator]
        ),

        TimerAction(
            period=11.0,
            actions=[mission_manager]
        ),

        TimerAction(
            period=13.0,
            actions=[qr_reader]
        ),
    ])
