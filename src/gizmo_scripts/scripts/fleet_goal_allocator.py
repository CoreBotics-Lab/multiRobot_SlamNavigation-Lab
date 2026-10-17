#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

# TF specific imports
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.duration import Duration

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

        # Initialize navigation nodes and publisher for empty paths for each robot
        self.nav = {}
        self.plan_pubs = {}
        for robot_name in self.robot_names:
            self.nav[robot_name] = BasicNavigator(node_name=f'{robot_name}_navigator', namespace=robot_name)
            self.nav[robot_name].set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)])
            self.plan_pubs[robot_name] = self.create_publisher(Path, f'{robot_name}/plan', 10)

        self.get_logger().info(f"Navigation nodes initialized: {self.nav}")

        # Intializing subscription to the 'goal_pose' topic
        self.get_goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)

        # tf2 buffer and listener initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # goal queueing
        self.goal_queue = [] 
        self.goal_queue_timer = self.create_timer(1.0, self.process_queue)

    def get_tf(self,robot_name: str) -> PoseStamped | None:
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform("map", f"{robot_name}/base_footprint",
                                                    Time(clock_type=ClockType.ROS_TIME),
                                                    timeout=Duration(seconds=1.0))

            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose

        except Exception as e:
            self.get_logger().warning(f"Could not get pose for {robot_name}: {e}")
            return None

    def calculate_path_length(self, path) -> float:
        if path is None or len(path.poses) < 2:
            return 0.0
        total_distance = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total_distance += math.hypot(p2.x - p1.x, p2.y - p1.y)
        return total_distance

    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Received goal: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")

        self.goal_queue.append(msg)  # Add the received goal to the queue
        self.get_logger().info(f"Goal added to queue. Total queued: {len(self.goal_queue)}")
        self.process_queue()

    def process_queue(self):
        if not self.goal_queue:
            return

        target_goal = self.goal_queue[0]

        # Reject goal if it is placed directly on top of ANY robot in the fleet
        min_clearance = 0.12  # meters (robot radius is 0.10m, so 0.12m gives safe cushion)
        for name in self.robot_names:
            pose = self.get_tf(name)
            if pose is not None:
                dist_to_bot = math.hypot(
                    target_goal.pose.position.x - pose.pose.position.x,
                    target_goal.pose.position.y - pose.pose.position.y
                )
                if dist_to_bot < min_clearance:
                    self.get_logger().warning(
                        f"Goal is too close to {name} ({dist_to_bot:.2f}m < {min_clearance}m). Rejecting goal."
                    )
                    self.goal_queue.pop(0)
                    return

        # Identify idle robots before getting their paths to the target goal. so that you dont have to calculate paths for busy robots.
        idle_robots = [name for name in self.robot_names if self.nav[name].isTaskComplete()]

        # idle_robots.clear() # This line is for testing purposes to simulate a scenario where no robots are idle.
        # idle_robots.pop(0)  # This line is for testing purposes to simulate a scenario where one robot is busy.
        if not idle_robots:
            # self.get_logger().info(f"No idle robots available to assign the goal.")
            return       

        # Calculate path lengths for all idle robots to the target goal
        path_lengths = {}
        for name in idle_robots:
            robot_pose = self.get_tf(name)
            if robot_pose is not None:
                get_path = self.nav[name].getPath(robot_pose, target_goal)
                if get_path is not None:
                    path_lengths[name] = self.calculate_path_length(get_path)
                    self.get_logger().info(f"Path length for {name}: {path_lengths[name]:.2f} meters")
                else:
                    self.get_logger().warning(f"Could not calculate path for {name}.")
            else:
                self.get_logger().warning(f"Could not get current pose for {name}.")

        if not path_lengths:
            self.get_logger().info("No valid paths found for any idle robots. Dropping unreachable goal.")
            self.goal_queue.pop(0)  # Remove the goal from the queue since no robot can take it
            return

        # Get the robot with the shortest path to the goal
        best_robot = min(path_lengths, key=lambda name: path_lengths[name])
        
        # Publish an empty path to all other robots to clear their previous plans
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()

        for name in self.robot_names:
            if name != best_robot:
                self.plan_pubs[name].publish(empty_path)

        # Assign the goal to the best robot
        self.get_logger().info(f"Assigning goal to {best_robot} with path length {path_lengths[best_robot]:.2f} meters")
        self.goal_queue.pop(0)
        self.nav[best_robot].goToPose(target_goal)

def main(args=None):
    rclpy.init(args=args)
    fleet_allocator_node = None  # Prevents UnboundLocalError if __init__ exits early

    try:
        fleet_allocator_node = fleet_goal_allocator()

        # Add ALL nodes to one executor:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(fleet_allocator_node)

        for nav_node in fleet_allocator_node.nav.values():
            executor.add_node(nav_node)

        executor.spin()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass  # Catches sys.exit(1) from parameter validation cleanly
    finally:
        try:
            if fleet_allocator_node is not None:
                fleet_allocator_node.get_logger().info("Shutting down nodes...")
                # Destroy child navigator nodes first
                if hasattr(fleet_allocator_node, 'nav'):
                    for nav_node in fleet_allocator_node.nav.values():
                        nav_node.destroy_node()
                # Destroy allocator node
                fleet_allocator_node.destroy_node()
        except Exception as e:
            get_logger('fleet_goal_allocator').error(f"Error occurred during teardown: {e}")

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
Terminal 1:
ros2 launch gizmo_bringup multi_robot_navigation.launch.py headless:=true robot_names:="robot1,robot2"

Terminal 2:
ros2 run gizmo_scripts fleet_goal_allocator.py --ros-args -p robot_names:="['robot1','robot2']"
"""