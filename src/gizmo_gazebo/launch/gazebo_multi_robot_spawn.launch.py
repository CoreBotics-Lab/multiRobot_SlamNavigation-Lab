#!/usr/bin/env python3

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_desc_dir = get_package_share_directory('gizmo_description')
    ros_gz_dir = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    run_rviz2 = LaunchConfiguration('run_rviz2').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    camera_type = LaunchConfiguration('camera_type').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    use_imu = LaunchConfiguration('use_imu').perform(context)

    ekf_template_path = os.path.join(gizmo_gazebo_dir, 'config', 'ekf_multi_robot.yaml')
    with open(ekf_template_path, 'r') as f:
        ekf_template_content = f.read()

    world_path = os.path.join(gizmo_gazebo_dir, 'worlds', 'simpleBiggerWorld.sdf')
    xacro_file = os.path.join(gizmo_desc_dir, 'urdf', 'gizmo.urdf.xacro')

    # Environment variables for Gazebo model loading
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.pathsep,
            os.path.dirname(gizmo_desc_dir)
        ]
    )

    headless_arg = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args_val = f'-r -s {world_path}' if headless_arg else f'-r {world_path}'

    # 1. Start Gazebo Sim with simpleBiggerWorld
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_dir, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': gz_args_val,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 2. Clock Bridge (Global)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    actions: list[Action] = [gz_resource_path, gz_sim, clock_bridge]

    # 3. Rviz2 Node
    if run_rviz2:
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        )
        actions.extend([rviz2])

    # 4. Load fleet configuration (robots with spawn coordinates)
    fleet_config_path = LaunchConfiguration('fleet_config').perform(context)
    robot_names_str = LaunchConfiguration('robot_names').perform(context).strip()
    with open(fleet_config_path, 'r') as f:
        fleet_data = yaml.safe_load(f)
    robots = fleet_data.get('robots', [])

    if robot_names_str and robot_names_str.lower() != 'all':
        allowed_names = {name.strip() for name in robot_names_str.split(',') if name.strip()}
        robots = [bot for bot in robots if bot.get('name') in allowed_names]

    for bot in robots:
        name: str = str(bot['name'])
        x_pos: str = str(bot.get('x', 0.0))
        y_pos: str = str(bot.get('y', 0.0))
        yaw: str = str(bot.get('yaw', 0.0))

        # Robot Description with prefix
        robot_description = ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' prefix:=', name,
                ' use_lidar:=', use_lidar,
                ' use_camera:=', use_camera,
                ' camera_type:=', camera_type,
                ' use_imu:=', use_imu
            ]),
            value_type=str
        )

        # Robot State Publisher under namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        )

        # Spawn entity in Gazebo
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', name,
                '-x', x_pos,
                '-y', y_pos,
                '-z', '0.04',
                '-Y', yaw,
                '-topic', f'/{name}/robot_description',
                '-world', 'empty'
            ]
        )

        # Ros-Gz Parameter Bridge for this robot
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge',
            output='screen',
            arguments=[
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera bridges:
                #camera_type:=camera
                f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                #camera_type:=depth_camera
                f'/{name}/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )

        # EKF Filter Node for this robot (generated from config/ekf.yaml)
        ekf_cfg_content = ekf_template_content.replace('{NAME}', name)
        temp_ekf = tempfile.NamedTemporaryFile(mode='w', prefix=f'ekf_{name}_', suffix='.yaml', delete=False)
        temp_ekf.write(ekf_cfg_content)
        temp_ekf.flush()
        temp_ekf.close()

        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=name,
            output='screen',
            parameters=[temp_ekf.name, {'use_sim_time': use_sim_time}],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        )

        actions.extend([rsp_node, spawn_node, bridge_node, ekf_node])

    return actions


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    default_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robots_fleet.yaml')

    launch_arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    launch_arg_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable/Disable Camera Sensor'
    )

    launch_arg_camera_type = DeclareLaunchArgument(
        'camera_type',
        default_value='camera',
        description="Gazebo camera plugin type: 'camera' or 'depth_camera'"
    )

    launch_arg_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable/Disable Lidar Sensor'
    )

    launch_arg_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable/Disable IMU Sensor'
    )

    headless_launch_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI if true'
    )

    fleet_config_arg = DeclareLaunchArgument(
        'fleet_config',
        default_value=default_fleet_config,
        description='Path to fleet configuration yaml defining robots and coordinates'
    )

    robot_names_arg = DeclareLaunchArgument(
        'robot_names',
        default_value='all',
        description='Comma-separated robot names to spawn (e.g. "robot1,robot2") or "all"'
    )

    launch_arg_run_rviz2 = DeclareLaunchArgument(
        'run_rviz2',
        default_value='false',
        description='Run RViz2 if true'
    )

    default_rviz_config = os.path.join(
        gizmo_gazebo_dir,
        'rviz',
        'gazebo.rviz'
    )

    launch_arg_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file.'
    )

    return LaunchDescription([
        launch_arg_use_sim_time,
        launch_arg_use_camera,
        launch_arg_camera_type,
        launch_arg_use_lidar,
        launch_arg_use_imu,
        launch_arg_run_rviz2,
        launch_arg_rviz_config,
        headless_launch_arg,
        fleet_config_arg,
        robot_names_arg,
        OpaqueFunction(function=launch_setup),
    ])
