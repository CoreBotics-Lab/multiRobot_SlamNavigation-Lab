#!/usr/bin/env python3

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_nav_dir = get_package_share_directory('gizmo_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    fleet_config_path = LaunchConfiguration('fleet_config').perform(context)
    nav2_params_path = LaunchConfiguration('nav2_params').perform(context)
    bt_xml_path = LaunchConfiguration('bt_xml').perform(context)
    robot_names_str = LaunchConfiguration('robot_names').perform(context).strip()

    # 1. Read fleet configuration
    with open(fleet_config_path, 'r') as f:
        fleet_data = yaml.safe_load(f)
    robots = fleet_data.get('robots', [])

    if robot_names_str and robot_names_str.lower() != 'all':
        allowed_names = {name.strip() for name in robot_names_str.split(',') if name.strip()}
        robots = [bot for bot in robots if bot.get('name') in allowed_names]

    # 2. Read Nav2 parameters template
    with open(nav2_params_path, 'r') as f:
        template_content = f.read()

    # 3. Global Map Server & Lifecycle Manager
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': use_sim_time
        }]
    )

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    actions: list[Action] = [map_server_node, lifecycle_manager_map_node]

    # 4. Per-Robot Nav2 Stack dynamically generated from fleet config
    shared_remappings = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
        ('map', '/map')
    ]

    for i, bot in enumerate(robots):
        name = str(bot['name'])
        x_pos = str(bot.get('x', 0.0))
        y_pos = str(bot.get('y', 0.0))
        yaw = str(bot.get('yaw', 0.0))
        groot_port = str(bot.get('groot_port', 3100 + i))

        robot_cfg_content = (
            template_content
            .replace('{NAME}', name)
            .replace('{X}', x_pos)
            .replace('{Y}', y_pos)
            .replace('{YAW}', yaw)
            .replace('{GROOT_PORT}', groot_port)
        )

        temp_cfg = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'nav2_{name}_', suffix='.yaml', delete=False
        )
        temp_cfg.write(robot_cfg_content)
        temp_cfg.flush()
        temp_cfg.close()
        cfg = temp_cfg.name

        localization_nodes = ['amcl']
        nav_nodes = [
            'controller_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower'
        ]

        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=name,
            output='screen',
            parameters=[cfg, {'use_sim_time': use_sim_time}],
            remappings=shared_remappings + [
                ('scan', f'/{name}/scan')
            ]
        )

        controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=name,
            output='screen',
            parameters=[cfg, {'use_sim_time': use_sim_time}],
            remappings=shared_remappings + [
                ('cmd_vel', f'/{name}/cmd_vel'),
                ('odom', f'/{name}/wheel_odom')
            ]
        )

        planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=name,
            output='screen',
            parameters=[cfg, {'use_sim_time': use_sim_time}],
            remappings=shared_remappings
        )

        behavior_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=name,
            output='screen',
            parameters=[cfg, {'use_sim_time': use_sim_time}],
            remappings=shared_remappings + [
                ('cmd_vel', f'/{name}/cmd_vel')
            ]
        )

        bt_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=name,
            output='screen',
            parameters=[
                cfg,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml_path
                }
            ],
            remappings=shared_remappings + [
                ('odom', f'/{name}/wheel_odom')
            ]
        )

        wp_node = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=name,
            output='screen',
            parameters=[cfg, {'use_sim_time': use_sim_time}],
            remappings=shared_remappings
        )

        lifecycle_mgr_loc = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': localization_nodes,
                'bond_timeout': 0.0,
                'service_timeout': 30.0,
                'attempt_respawn_reconnection': True
            }]
        )

        lifecycle_mgr_nav = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': nav_nodes,
                'bond_timeout': 0.0,
                'service_timeout': 30.0,
                'attempt_respawn_reconnection': True
            }]
        )

        # 3.0-second delay gives AMCL time to initialize with initial pose and publish map->odom TF
        delayed_nav_mgr = TimerAction(
            period=3.0,
            actions=[lifecycle_mgr_nav]
        )

        robot_group = GroupAction(
            actions=[
                amcl_node,
                controller_node,
                planner_node,
                behavior_node,
                bt_node,
                wp_node,
                lifecycle_mgr_loc,
                delayed_nav_mgr
            ]
        )

        actions.extend([robot_group])

    return actions


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_nav_dir = get_package_share_directory('gizmo_navigation')

    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')
    default_map = os.path.join(gizmo_mapping_dir, 'maps', 'simple_bigger_world_map.yaml')
    default_nav2_params = os.path.join(gizmo_nav_dir, 'config', 'nav2_params.yaml')
    default_bt_xml = os.path.join(gizmo_nav_dir, 'behavior_trees', 'gizmo_nav_to_pose.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=default_fleet_config,
            description='Full path to fleet configuration yaml'
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=default_nav2_params,
            description='Full path to base nav2 parameters template yaml'
        ),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=default_bt_xml,
            description='Full path to behavior tree XML file'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='all',
            description='Comma-separated robot names to run Nav2 for (e.g. "robot1,robot2") or "all"'
        ),
        OpaqueFunction(function=launch_setup)
    ])

