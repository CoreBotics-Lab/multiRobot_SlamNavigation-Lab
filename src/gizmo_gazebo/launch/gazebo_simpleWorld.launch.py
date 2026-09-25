import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    
    simple_world_path = os.path.join(
        gizmo_gazebo_dir,
        'worlds',
        'simpleWorld.sdf'
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_file': simple_world_path,
            'use_camera': 'false',
            'run_rviz2': 'false'
        }.items()
    )
    
    return LaunchDescription([
        gazebo_launch
    ])
