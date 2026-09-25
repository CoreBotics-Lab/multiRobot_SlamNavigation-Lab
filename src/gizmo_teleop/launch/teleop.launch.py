import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare Launch Arguments
    launch_arg_topic_name = DeclareLaunchArgument(
        'topic_name',
        default_value='cmd_vel',
        description='Topic to publish Twist/TwistStamped velocity commands to'
    )

    launch_arg_max_linear = DeclareLaunchArgument(
        'max_linear',
        default_value='0.73',
        description='Max linear velocity limit in m/s (based on 200 RPM, 70mm wheel)'
    )

    launch_arg_max_angular = DeclareLaunchArgument(
        'max_angular',
        default_value=str(math.pi),
        description='Max angular velocity limit in rad/s (pi rad/s = 180 deg/sec)'
    )

    launch_arg_twist_stamped = DeclareLaunchArgument(
        'twistStamped',
        default_value='false',
        description='Publish geometry_msgs/TwistStamped instead of geometry_msgs/Twist'
    )

    launch_arg_publish_rate_hz = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Cmd_vel publishing rate in Hz'
    )

    # Node configuration
    teleop_gui_node = Node(
        package='gizmo_teleop',
        executable='joystick_gui',
        name='joy_gui_node',
        output='screen',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
            'twistStamped': LaunchConfiguration('twistStamped'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }]
    )

    return LaunchDescription([
        launch_arg_topic_name,
        launch_arg_max_linear,
        launch_arg_max_angular,
        launch_arg_twist_stamped,
        launch_arg_publish_rate_hz,
        teleop_gui_node
    ])
