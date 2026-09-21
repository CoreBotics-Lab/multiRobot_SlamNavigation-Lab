from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gizmo_description_dir = FindPackageShare('gizmo_description')
    xacro_file = PathJoinSubstitution([gizmo_description_dir, 'urdf', 'gizmo.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([gizmo_description_dir, 'rviz', 'display.rviz'])

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for robot frames and topics (e.g. robot1)'
    )
    prefix = LaunchConfiguration('prefix')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        prefix_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
