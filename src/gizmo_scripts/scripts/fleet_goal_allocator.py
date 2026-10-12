#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException

class fleet_goal_allocator(Node):
    def __init__(self):
        super().__init__('fleet_goal_allocator')
        self.declare_parameter('robot_names', rclpy.Parameter.Type.STRING_ARRAY)

        try:
            param = self.get_parameter('robot_names')
            self.robot_names = list(param.get_parameter_value().string_array_value)
            for i in range(len(self.robot_names)):
                print(f"Robot names: {self.robot_names[i]}")

        except (ParameterUninitializedException, ValueError) as e:
            print(f"Error retrieving parameter 'robot_names': {e}")
            self.destroy_node()
            exit(1)

def main(args=None):
    rclpy.init(args=args)
    fleet_allocator_node = fleet_goal_allocator()

    try:
        rclpy.spin(fleet_allocator_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            fleet_allocator_node.destroy_node()
        except Exception as e:
            print(f"Error occurred while destroying node: {e}")
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()