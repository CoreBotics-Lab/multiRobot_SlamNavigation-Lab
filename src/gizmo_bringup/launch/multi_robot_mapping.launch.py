#!/usr/bin/env python3

import os
import yaml # type: ignore
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
    headless = LaunchConfiguration('headless').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    robot_names = LaunchConfiguration('robot_names').perform(context)
    publish_static_map_tf = LaunchConfiguration('publish_static_map_tf').perform(context)
    run_rviz2 = LaunchConfiguration('run_rviz2').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    gazebo_delay = float(LaunchConfiguration('gazebo_delay').perform(context))

    run_teleop = LaunchConfiguration('run_teleop').perform(context)

    # Resolve active robot names from fleet configuration
    with open(fleet_config, 'r') as f:
        fleet_data = yaml.safe_load(f)
    robots = fleet_data.get('robots', [])
    robot_names_str = robot_names.strip()
    if robot_names_str and robot_names_str.lower() != 'all':
        allowed = {n.strip() for n in robot_names_str.split(',') if n.strip()}
        active_robot_names = [bot['name'] for bot in robots if bot.get('name') in allowed]
    else:
        active_robot_names = [bot['name'] for bot in robots]


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

    # 2. Multi-Robot SLAM (spawns namespaced SLAM Toolbox and Lifecycle Manager per robot)
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

    # 3. Multi-Robot Map Merge (fuses /{name}/map into unified /map)
    map_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_mapping_dir, 'launch', 'map_merge.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'fleet_config': fleet_config,
            'robot_names': robot_names,
            'merged_map_topic': '/map',
            'rate': '1.0',
        }.items()
    )

    remaining_actions: list[Action] = [multi_slam_launch, map_merge_launch]

    # 4. RViz2 Mapping Visualization Dashboard
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

    # Wait until Gazebo has completely loaded and spawned robots before starting the rest of the stack
    delayed_mapping_stack = TimerAction(
        period=gazebo_delay,
        actions=remaining_actions
    )

    actions: list[Action] = [gazebo_spawn_launch, delayed_mapping_stack]

    # 4. teleop node for controlling the robots

    if run_teleop.lower() == 'true':
        for name in active_robot_names:
            teleop_gui_node = Node(
                package='gizmo_teleop',
                executable='joystick_gui',
                name=f'joy_{name}_node',
                output='screen',
                parameters=[{
                    'topic_name': f'/{name}/cmd_vel',
                    'max_linear': 0.73,
                    'max_angular': 3.14159,
                    'twistStamped': False,
                    'publish_rate_hz': 10.0,
                }]
            )
            actions.extend([teleop_gui_node])
        
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
        DeclareLaunchArgument(
            'gazebo_delay',
            default_value='7.0',
            description='Delay in seconds to wait for Gazebo to completely load and spawn robots before launching SLAM, map merge, and RViz2'
        ),
        DeclareLaunchArgument(
            'run_teleop',
            default_value='true',
            description='Launch the joystick GUI node automatically'
        ),
        OpaqueFunction(function=launch_setup),
    ])
