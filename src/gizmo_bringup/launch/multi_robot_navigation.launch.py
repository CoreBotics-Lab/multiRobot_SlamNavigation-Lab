#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_nav_dir = get_package_share_directory('gizmo_navigation')

    cyclonedds_config = os.path.join(gizmo_bringup_dir, 'config', 'cyclonedds.xml')
    set_cyclonedds_uri = SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_config)
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    robot_names = LaunchConfiguration('robot_names').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    run_rviz2 = LaunchConfiguration('run_rviz2').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    gazebo_delay = float(LaunchConfiguration('gazebo_delay').perform(context))

    # 1. Gazebo Multi-Robot Spawn (spawns Gazebo, clock bridge, RSP, ROS-Gz bridges, EKF)
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo_multi_robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'fleet_config': fleet_config,
            'robot_names': robot_names,
        }.items()
    )

    # 2. Multi-Robot Navigation (spawns map_server, AMCL, and Nav2 stack per robot)
    multi_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_nav_dir, 'launch', 'multi_robot_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'fleet_config': fleet_config,
            'robot_names': robot_names,
            'map': map_file,
            'autostart': 'true',
        }.items()
    )

    remaining_actions: list[Action] = [multi_nav_launch]

    # 3. RViz2 Navigation Visualization Dashboard
    if run_rviz2.lower() == 'true':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}]
        )
        remaining_actions.extend([rviz_node])

    # Wait until Gazebo has completely loaded and spawned robots before starting navigation & RViz2
    delayed_nav_stack = TimerAction(
        period=gazebo_delay,
        actions=remaining_actions
    )

    return [set_rmw, set_cyclonedds_uri, gazebo_spawn_launch, delayed_nav_stack]


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_nav_dir = get_package_share_directory('gizmo_navigation')

    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')
    default_map = os.path.join(gizmo_mapping_dir, 'maps', 'simple_bigger_world_map.yaml')
    default_rviz_config = os.path.join(gizmo_nav_dir, 'rviz', 'multi_robot_navigation.rviz')

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
            description='Comma-separated robot names to run Nav2 for (e.g. "robot1,robot2") or "all"'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map YAML file'
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
            default_value='7.0',
            description='Delay in seconds to wait for Gazebo to completely load and spawn robots before launching navigation and RViz2'
        ),
        OpaqueFunction(function=launch_setup),
    ])

