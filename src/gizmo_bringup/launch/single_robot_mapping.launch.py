import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simpleBiggerWorld.sdf',
        description='World file name inside gizmo_gazebo/worlds/ (e.g. simpleWorld.sdf, simpleBiggerWorld.sdf)'
    )
    world = LaunchConfiguration('world')

    default_slam_params = os.path.join(
        gizmo_mapping_dir,
        'config',
        'slam_toolbox_params.yaml'
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the ROS 2 parameters file for slam_toolbox'
    )
    slam_params_file = LaunchConfiguration('slam_params_file')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the SLAM lifecycle stack'
    )
    autostart = LaunchConfiguration('autostart')

    run_rviz2_arg = DeclareLaunchArgument(
        'run_rviz2',
        default_value='true',
        description='Run RViz2 if true'
    )
    run_rviz2 = LaunchConfiguration('run_rviz2')

    default_rviz_config = os.path.join(
        gizmo_bringup_dir,
        'rviz',
        'slam.rviz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # 2. Paths to building block launch files
    gazebo_launch_path = os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo.launch.py')
    slam_launch_path = os.path.join(gizmo_mapping_dir, 'launch', 'slam.launch.py')

    world_path = PathJoinSubstitution([
        FindPackageShare('gizmo_gazebo'),
        'worlds',
        world
    ])

    # 3. Gazebo Simulation Launch (Spawns Gizmo, robot_state_publisher, bridges, EKF, and RViz2)
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world_file': world_path,
            'use_camera': 'false',
            'run_rviz2': run_rviz2,
            'rviz_config': rviz_config,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. SLAM Toolbox Launch (async_slam_toolbox_node + nav2_lifecycle_manager)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'autostart': autostart
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        slam_params_file_arg,
        autostart_arg,
        run_rviz2_arg,
        rviz_config_arg,
        gazebo_simulation,
        slam_launch
    ])
