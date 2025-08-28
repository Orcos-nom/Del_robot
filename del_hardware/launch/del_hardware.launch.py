import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Expand XACRO at runtime for real‑hardware URDF (no Gazebo)
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("del_description"),
                    "urdf",
                    "del.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    # Controller config YAML (real hardware)
    controller_params = PathJoinSubstitution([
        FindPackageShare('del_controller'),
        'config',
        'del_controllers.yaml'
    ])

    # 1 Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}]
    )

    # 2 Start controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False},
                    controller_params]
    )

    # 3 Spawn joint_state_broadcaster
    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    # 4 Spawn diff_drive_base_controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--param-file', controller_params]
    )

    # Sequence with small delays
    delayed_mgr = TimerAction(period=2.0, actions=[controller_manager])
    delayed_jsb = TimerAction(period=4.0, actions=[joint_broadcaster])
    delayed_dd  = TimerAction(period=6.0, actions=[diff_drive_spawner])

    return LaunchDescription([
        robot_state_publisher,
        delayed_mgr,
        delayed_jsb,
        delayed_dd
    ])
