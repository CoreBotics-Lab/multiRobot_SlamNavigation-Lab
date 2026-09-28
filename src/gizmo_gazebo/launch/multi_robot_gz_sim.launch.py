"""Launch file for multi robot simulation in Gazebo."
Common:
 - world
 - clockbridge
 - gz resource path
 - gz sim

for each robot:
 - robot description
 - robot state publisher
 - spawn node
 - ros gz bridge for each topics except clock
 - ekf node

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def launch_setup(context, *args, **kwargs):

    gizmo_desc_dir = get_package_share_directory('gizmo_description')
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(gizmo_gazebo_dir, 'worlds', 'simpleWorld.sdf')
    robot_fleet_config = os.path.join(gizmo_gazebo_dir, 'config', 'robot_fleet.yaml')
    ekf_config_file = os.path.join(gizmo_gazebo_dir, 'config', 'ekf_multi_robot.yaml')
    xacro_file = os.path.join(gizmo_desc_dir, 'urdf', 'gizmo.urdf.xacro')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.pathsep,
            os.path.dirname(gizmo_desc_dir)
        ]
    )

    # headless_arg = LaunchConfiguration('headless').perform(context).lower() == 'true'
    headless_arg = False

    gz_args_val = f'-r -s {world_file}' if headless_arg else f'-r {world_file}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_sim_dir, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': gz_args_val,
            'on_exit_shutdown': 'true'
        }.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}]
    )

    # actions : list[Action] = [gz_resource_path, gz_sim, clock_bridge]
    actions: list[Action] = []

    with open(robot_fleet_config, 'r') as f:
        fleet_data = yaml.safe_load(f)
    robots = fleet_data.get('robots', [])

    for bot in robots:
        name: str = str(bot['name'])
        x_pos: str = str(bot.get('x', 0.0))
        y_pos: str = str(bot.get('y', 0.0))
        yaw: str = str(bot.get('yaw', 0.0))
        config = f"{name}: x: {x_pos} y: {y_pos} yaw: {yaw}\n"

        print(config)
    
    return actions


def generate_launch_description():



    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])