#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    robot_names = LaunchConfiguration('robot_names').perform(context)
    publish_static_map_tf = LaunchConfiguration('publish_static_map_tf').perform(context)
    run_rviz2 = LaunchConfiguration('run_rviz2').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    # 1. Gazebo Multi-Robot Spawn (spawns Gazebo, clock bridge, RSP, ROS-Gz bridges, EKF, and RViz2)
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo_multi_robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'fleet_config': fleet_config,
            'robot_names': robot_names,
            'run_rviz2': run_rviz2,
            'rviz_config': rviz_config,
        }.items()
    )

    # 2. Multi-Robot SLAM (spawns namespaced SLAM Toolbox and Lifecycle Manager per robot)
    # Delayed slightly to let Gazebo, bridges, RSP, and EKF settle before lifecycle transitions start
    multi_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_mapping_dir, 'launch', 'multi_robot_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'fleet_config': fleet_config,
            'robot_names': robot_names,
            'publish_static_map_tf': publish_static_map_tf,
            'autostart': 'true',
        }.items()
    )

    delayed_slam_launch = TimerAction(
        period=4.0,
        actions=[multi_slam_launch]
    )

    actions: list[Action] = [gazebo_spawn_launch, delayed_slam_launch]

    return actions


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')
    default_rviz_config = os.path.join(gizmo_mapping_dir, 'rviz', 'multi_robot_slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode without GUI if true'
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=default_fleet_config,
            description='Full path to robots_fleet.yaml defining robot names and spawn poses'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='all',
            description='Comma-separated robot names to run SLAM for (e.g. "robot1,robot2") or "all"'
        ),
        DeclareLaunchArgument(
            'publish_static_map_tf',
            default_value='true',
            description='Publish static initial TF from "map" to "{name}/map" for RViz visualization before map merge'
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
        OpaqueFunction(function=launch_setup),
    ])
