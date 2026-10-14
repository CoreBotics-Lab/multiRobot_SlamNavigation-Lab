#!/usr/bin/env python3
import sys
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped

class fleet_goal_allocator(Node):
    def __init__(self):
        super().__init__('fleet_goal_allocator')

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        # Declare and retrieve the 'robot_names' parameter
        self.declare_parameter('robot_names', rclpy.Parameter.Type.STRING_ARRAY)
        try:
            param = self.get_parameter('robot_names')
            self.robot_names = list(param.get_parameter_value().string_array_value)
            for name in self.robot_names:
                self.get_logger().info(f"Robot names: {name}")

        except (ParameterUninitializedException, ValueError) as e:
            self.get_logger().error(f"Error retrieving parameter 'robot_names': {e}")
            self.destroy_node()
            sys.exit(1)

        # Initialize navigation nodes and status dictionary for each robot
        self.nav = {}
        self.status = {}
        for robot_name in self.robot_names:
            self.nav[robot_name] = BasicNavigator(node_name=f'{robot_name}_navigator', namespace=robot_name)
            self.status[robot_name] = 'IDLE'
            self.nav[robot_name].set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)])
            
        self.get_logger().info(f"Navigation nodes initialized: {self.nav}")
        self.get_logger().info(f"Robot statuses initialized: {self.status}")

        # Intializing subscription to the 'goal_pose' topic
        self.get_goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)

    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Received goal: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    fleet_allocator_node = fleet_goal_allocator()

    # Add ALL nodes to one executor:
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(fleet_allocator_node)

    for nav_node in fleet_allocator_node.nav.values():
        executor.add_node(nav_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            fleet_allocator_node.destroy_node()
            for nav_node in fleet_allocator_node.nav.values():
                nav_node.destroy_node()
        except Exception as e:
            get_logger('fleet_goal_allocator').error(f"Error occurred: {e}")
       
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()