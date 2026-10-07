#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    fleet_config_arg = DeclareLaunchArgument(
        'fleet_config',
        default_value=default_fleet_config,
        description='Full path to robots_fleet.yaml defining robot names and spawn poses'
    )
    robot_names_arg = DeclareLaunchArgument(
        'robot_names',
        default_value='all',
        description='Comma-separated robot names to merge (e.g. "robot1,robot2") or "all"'
    )
    merged_map_topic_arg = DeclareLaunchArgument(
        'merged_map_topic',
        default_value='/map',
        description='Output topic for the fused global occupancy grid'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Map merge update frequency in Hz'
    )

    map_merge_node = Node(
        package='gizmo_mapping',
        executable='map_merge_node.py',
        name='map_merge_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'fleet_config': LaunchConfiguration('fleet_config'),
            'robot_names': LaunchConfiguration('robot_names'),
            'merged_map_topic': LaunchConfiguration('merged_map_topic'),
            'rate': LaunchConfiguration('rate')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        fleet_config_arg,
        robot_names_arg,
        merged_map_topic_arg,
        rate_arg,
        map_merge_node,
    ])
