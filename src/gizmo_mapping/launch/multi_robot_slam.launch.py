#!/usr/bin/env python3

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    fleet_config_path = LaunchConfiguration('fleet_config').perform(context)
    slam_template_path = LaunchConfiguration('slam_template').perform(context)
    robot_names_str = LaunchConfiguration('robot_names').perform(context).strip()
    publish_static_map_tf = LaunchConfiguration('publish_static_map_tf').perform(context).lower() == 'true'

    # 1. Read SLAM Toolbox template parameter file
    with open(slam_template_path, 'r') as f:
        slam_template_content = f.read()

    # 2. Read fleet configuration
    with open(fleet_config_path, 'r') as f:
        fleet_data = yaml.safe_load(f)
    robots = fleet_data.get('robots', [])

    # Filter robots if robot_names is specified (e.g. "robot1,robot2" or "all")
    if robot_names_str and robot_names_str.lower() != 'all':
        allowed_names = {name.strip() for name in robot_names_str.split(',') if name.strip()}
        robots = [bot for bot in robots if bot.get('name') in allowed_names]

    actions: list[Action] = []

    # 3. Spawn SLAM Toolbox and Lifecycle Manager for each robot
    for bot in robots:
        name = str(bot['name'])
        x_pos = str(bot.get('x', 0.0))
        y_pos = str(bot.get('y', 0.0))
        yaw = str(bot.get('yaw', 0.0))

        # Generate per-robot SLAM parameters
        slam_cfg_content = slam_template_content.replace('{NAME}', name)
        temp_slam = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'slam_{name}_', suffix='.yaml', delete=False
        )
        temp_slam.write(slam_cfg_content)
        temp_slam.flush()
        temp_slam.close()

        # Async SLAM Toolbox node
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=name,
            output='screen',
            parameters=[
                temp_slam.name,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('map', f'/{name}/map'),
                ('/map', f'/{name}/map'),
                ('map_metadata', f'/{name}/map_metadata'),
                ('/map_metadata', f'/{name}/map_metadata'),
                ('map_updates', f'/{name}/map_updates'),
                ('/map_updates', f'/{name}/map_updates'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        )

        # Nav2 Lifecycle Manager for this robot's SLAM node
        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
                'service_timeout': 30.0,
                'attempt_respawn_reconnection': True
            }]
        )

        actions.extend([slam_node, lifecycle_node])

        # Optional static TF broadcaster: connects master 'map' -> '{name}/map'
        # Useful for RViz visualization before map merge
        if publish_static_map_tf:
            static_tf_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_map_to_{name}_map',
                output='screen',
                arguments=[
                    '--x', x_pos,
                    '--y', y_pos,
                    '--z', '0.0',
                    '--yaw', yaw,
                    '--frame-id', 'map',
                    '--child-frame-id', f'{name}/map'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
            actions.extend([static_tf_node])

    return actions


def generate_launch_description():
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')

    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')
    default_slam_template = os.path.join(gizmo_mapping_dir, 'config', 'slam_toolbox_multi_robot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the SLAM lifecycle stack'
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=default_fleet_config,
            description='Full path to robots_fleet.yaml defining robot names and spawn poses'
        ),
        DeclareLaunchArgument(
            'slam_template',
            default_value=default_slam_template,
            description='Full path to SLAM Toolbox multi-robot template configuration'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='all',
            description='Comma-separated robot names to run SLAM for (e.g. "robot1,robot2") or "all"'
        ),
        DeclareLaunchArgument(
            'publish_static_map_tf',
            default_value='false',
            description='Publish static initial TF from "map" to "{name}/map" for RViz alignment before map merge'
        ),
        OpaqueFunction(function=launch_setup),
    ])
