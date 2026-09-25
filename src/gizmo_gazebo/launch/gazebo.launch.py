import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_description_dir = get_package_share_directory('gizmo_description')
    ros_gz_package_dir = get_package_share_directory('ros_gz_sim')

    launch_arg_use_sim_true = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_arg_run_rviz2 = DeclareLaunchArgument(
        'run_rviz2',
        default_value='true',
        description='Run RViz2 if true'
    )
    run_rviz2 = LaunchConfiguration('run_rviz2')

    launch_arg_headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo without the GUI'
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
    rviz_config = LaunchConfiguration('rviz_config')

    launch_arg_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='empty.sdf',
        description='Gazebo world file name or full path (e.g. empty.sdf).'
    )
    world_file = LaunchConfiguration('world_file')

    launch_arg_camera_type = DeclareLaunchArgument(
        'camera_type',
        default_value='camera',
        description="Gazebo camera sensor plugin type. Options: 'camera' (2D RGB) or 'depth_camera' (3D RGB-D / PointCloud2)."
    )
    camera_type = LaunchConfiguration('camera_type')

    launch_arg_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable or disable Gazebo lidar plugin'
    )
    use_lidar = LaunchConfiguration('use_lidar')

    launch_arg_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable or disable Gazebo camera plugin'
    )
    use_camera = LaunchConfiguration('use_camera')

    launch_arg_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable or disable Gazebo IMU plugin'
    )
    use_imu = LaunchConfiguration('use_imu')

    gizmo_parent_dir = os.path.dirname(gizmo_description_dir)

    env_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.pathsep,
            gizmo_parent_dir
        ]
    )

    xacro_file_path = os.path.join(gizmo_description_dir, 'urdf', 'gizmo.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file_path,
            ' camera_type:=', camera_type,
            ' use_lidar:=', use_lidar,
            ' use_camera:=', use_camera,
            ' use_imu:=', use_imu
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time}
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_group = GroupAction(
        condition=IfCondition(run_rviz2),
        actions=[
            launch_arg_rviz_config,
            rviz2
        ]
    )

    bridge_config_file = os.path.join(
        gizmo_gazebo_dir,
        'config', 'ros_gz_bridge_config.yaml'
    )

    gazebo_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        name='clock_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': use_sim_time
        }]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_package_dir, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "'.lower() == 'true' else ''"]),
                world_file
            ],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'gizmo',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-topic', 'robot_description',
            '-world', 'empty'
        ],
        output='screen'
    )

    ekf_config_file = os.path.join(
        gizmo_gazebo_dir,
        'config', 'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        launch_arg_use_sim_true,
        launch_arg_run_rviz2,
        launch_arg_headless,
        launch_arg_world_file,
        launch_arg_camera_type,
        launch_arg_use_lidar,
        launch_arg_use_camera,
        launch_arg_use_imu,
        env_gz_resource_path,
        robot_state_publisher,
        rviz_group,
        gazebo_clock_bridge,
        gz_sim,
        gz_spawn_entity,
        ekf_node
    ])