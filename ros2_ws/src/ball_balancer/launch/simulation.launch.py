import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('ball_balancer')

    # ------------------------------------------------------------------ #
    # Process xacro → URDF string                                         #
    # ------------------------------------------------------------------ #
    xacro_path = os.path.join(pkg_path, 'urdf', 'ball_beam.urdf.xacro')
    robot_desc = xacro.process_file(xacro_path).toxml()

    # ------------------------------------------------------------------ #
    # Nodes                                                               #
    # ------------------------------------------------------------------ #

    # 1. Gazebo Harmonic (headless flag removed so you can see the GUI)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # 2. robot_state_publisher  (publishes /tf from joint states)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
        output='screen'
    )

    # 3. Spawn robot into Gazebo (slight delay so gz sim is ready)
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', robot_desc,
                    '-name',   'ball_beam',
                    '-z',      '0.05',   # spawn slightly above ground
                ],
                output='screen'
            )
        ]
    )

    # 4. joint_state_broadcaster spawner
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 5. beam_position_controller spawner  (after jsb is active)
    beam_ctrl_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['beam_position_controller',
                               '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        )
    )

    # 6. ros_gz_bridge  – clock so nodes can use sim time
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        gz_bridge,
        jsb_spawner,
        beam_ctrl_spawner,
    ])

