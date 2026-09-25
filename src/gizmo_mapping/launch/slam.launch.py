import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for multi-robot setup'
    )
    namespace = LaunchConfiguration('namespace')

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

    # 2. Lifecycle nodes for SLAM
    lifecycle_nodes = ['slam_toolbox']

    # 3. SLAM Toolbox Node (Online Asynchronous)
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. Lifecycle Manager for SLAM
    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        slam_params_file_arg,
        autostart_arg,
        start_async_slam_toolbox_node,
        start_lifecycle_manager
    ])
