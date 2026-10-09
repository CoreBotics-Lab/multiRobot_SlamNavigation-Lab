#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    world = LaunchConfiguration('world').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    run_rviz2 = LaunchConfiguration('run_rviz2').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    gazebo_delay = float(LaunchConfiguration('gazebo_delay').perform(context))

    world_path = os.path.join(gizmo_gazebo_dir, 'worlds', world)
    gazebo_launch_path = os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo.launch.py')
    slam_launch_path = os.path.join(gizmo_mapping_dir, 'launch', 'slam.launch.py')

    # 1. Gazebo Simulation Launch (Spawns Gizmo, robot_state_publisher, bridges, EKF)
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world_file': world_path,
            'use_camera': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 2. SLAM Toolbox Launch (async_slam_toolbox_node + nav2_lifecycle_manager)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'autostart': autostart,
        }.items()
    )

    remaining_actions: list[Action] = [slam_launch]

    # 3. RViz2 Visualization Dashboard
    if run_rviz2.lower() == 'true':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}]
        )
        remaining_actions.extend([rviz_node])

    # Wait until Gazebo has completely loaded and spawned the robot before starting SLAM & RViz
    delayed_mapping_stack = TimerAction(
        period=gazebo_delay,
        actions=remaining_actions
    )

    return [gazebo_simulation, delayed_mapping_stack]


def generate_launch_description():
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    default_slam_params = os.path.join(
        gizmo_mapping_dir,
        'config',
        'slam_toolbox_params.yaml'
    )
    default_rviz_config = os.path.join(
        gizmo_bringup_dir,
        'rviz',
        'slam.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simpleBiggerWorld.sdf',
            description='World file name inside gizmo_gazebo/worlds/ (e.g. simpleWorld.sdf, simpleBiggerWorld.sdf)'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to the ROS 2 parameters file for slam_toolbox'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the SLAM lifecycle stack'
        ),
        DeclareLaunchArgument(
            'run_rviz2',
            default_value='true',
            description='Run RViz2 if true'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to the RViz configuration file'
        ),
        DeclareLaunchArgument(
            'gazebo_delay',
            default_value='5.0',
            description='Delay in seconds to wait for Gazebo to load before launching SLAM and RViz2'
        ),
        OpaqueFunction(function=launch_setup),
    ])
