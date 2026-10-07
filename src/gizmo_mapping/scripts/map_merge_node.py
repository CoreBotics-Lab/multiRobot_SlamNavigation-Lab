#!/usr/bin/env python3
"""
Custom Multi-Robot Map Merging Node for Gizmo Fleet.

Listens to /{name}/map for each active robot, transforms the 2D occupancy
grids using known spawn coordinates from robots_fleet.yaml, and fuses them
into a single unified global occupancy grid published on /map.
"""

import math
import os
import numpy as np
import yaml  # type: ignore

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


from dataclasses import dataclass


@dataclass
class PreparedMap:
    robot_name: str
    grid_data: np.ndarray
    timestamp: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    spawn_offset_x: float
    spawn_offset_y: float
    spawn_yaw_offset: float
    cos_yaw: float
    sin_yaw: float


class MultiRobotMapMergeNode(Node):

    def __init__(self):
        super().__init__('map_merge_node')

        # 1. Declare Parameters
        self.declare_parameter('fleet_config', '')
        self.declare_parameter('robot_names', 'all')
        self.declare_parameter('merged_map_topic', '/map')
        self.declare_parameter('rate', 1.0)  # Publish merged map every 1.0s

        fleet_config_path = self.get_parameter('fleet_config').value
        robot_names_str = self.get_parameter('robot_names').value
        merged_map_topic = self.get_parameter('merged_map_topic').value
        rate = float(self.get_parameter('rate').value)

        # 2. Parse Fleet Configuration
        self.robot_offsets = {}  # {name: (x, y, yaw)}
        if fleet_config_path and os.path.exists(fleet_config_path):
            with open(fleet_config_path, 'r') as f:
                data = yaml.safe_load(f)
            robots = data.get('robots', [])
            if robot_names_str and robot_names_str.lower() != 'all':
                allowed = {n.strip() for n in robot_names_str.split(',') if n.strip()}
                robots = [r for r in robots if r.get('name') in allowed]

            for r in robots:
                name = str(r['name'])
                x = float(r.get('x', 0.0))
                y = float(r.get('y', 0.0))
                yaw = float(r.get('yaw', 0.0))
                self.robot_offsets[name] = (x, y, yaw)
                self.get_logger().info(f"Registered robot '{name}' at spawn offset: ({x:.2f}, {y:.2f}, yaw: {yaw:.2f})")
        else:
            self.get_logger().error(f"fleet_config file not found: {fleet_config_path}")

        # 3. Store Latest Received Local Maps
        self.local_maps = {}  # {name: OccupancyGrid}

        # 4. QoS Profile matching SLAM Toolbox (Transient Local)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 5. Subscribers per Robot
        self.subs = []
        for name in self.robot_offsets.keys():
            topic = f'/{name}/map'
            sub = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, n=name: self.map_callback(msg, n),
                map_qos
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to map topic: {topic}")

        # 6. Publisher for Unified Global Map
        self.merged_pub = self.create_publisher(OccupancyGrid, merged_map_topic, map_qos)

        # 7. Periodic Merging Timer
        self.timer = self.create_timer(1.0 / rate, self.merge_and_publish)
        self.get_logger().info(f"Map merge node initialized. Publishing on '{merged_map_topic}' at {rate} Hz.")

    def map_callback(self, msg: OccupancyGrid, robot_name: str):
        """Save latest map received for a given robot."""
        self.local_maps[robot_name] = msg

    def merge_and_publish(self):
        """Fuse all received local occupancy grids onto a common canvas."""
        if not self.local_maps:
            return  # No maps received yet

        # 1. Determine common resolution and bounding box in global world meters
        valid_maps = []
        for name, grid in self.local_maps.items():
            if grid.info.width > 0 and grid.info.height > 0:
                valid_maps.append((name, grid))

        if not valid_maps:
            return

        resolution = valid_maps[0][1].info.resolution

        # Compute bounding box of all maps placed in world coordinates
        all_corners_x = []
        all_corners_y = []

        prepared_maps = []
        for name, grid in valid_maps:
            spawn_offset_x, spawn_offset_y, spawn_yaw_offset = self.robot_offsets.get(name, (0.0, 0.0, 0.0))
            
            # Local grid origin (bottom-left)
            origin_x = grid.info.origin.position.x
            origin_y = grid.info.origin.position.y
            width = grid.info.width
            height = grid.info.height

            # Four corners of this local map in its own frame
            corners_local = np.array([
                [origin_x, origin_y],
                [origin_x + width * resolution, origin_y],
                [origin_x + width * resolution, origin_y + height * resolution],
                [origin_x, origin_y + height * resolution]
            ])

            # Rotate and translate by robot spawn offset
            cos_yaw = math.cos(spawn_yaw_offset)
            sin_yaw = math.sin(spawn_yaw_offset)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            corners_world = (rotation_matrix @ corners_local.T).T + np.array([spawn_offset_x, spawn_offset_y])

            all_corners_x.extend(corners_world[:, 0])
            all_corners_y.extend(corners_world[:, 1])

            grid_data = np.array(grid.data, dtype=np.int8).reshape((height, width))
            timestamp = grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9
            prepared_maps.append(PreparedMap(
                robot_name=name,
                grid_data=grid_data,
                timestamp=timestamp,
                width=width,
                height=height,
                origin_x=origin_x,
                origin_y=origin_y,
                spawn_offset_x=spawn_offset_x,
                spawn_offset_y=spawn_offset_y,
                spawn_yaw_offset=spawn_yaw_offset,
                cos_yaw=cos_yaw,
                sin_yaw=sin_yaw,
            ))

        # Sort so newer observations are processed after older ones (freshest data updates the canvas)
        prepared_maps.sort(key=lambda robot_map: robot_map.timestamp)

        min_x = math.floor(min(all_corners_x) / resolution) * resolution
        max_x = math.ceil(max(all_corners_x) / resolution) * resolution
        min_y = math.floor(min(all_corners_y) / resolution) * resolution
        max_y = math.ceil(max(all_corners_y) / resolution) * resolution

        global_w = int(round((max_x - min_x) / resolution))
        global_h = int(round((max_y - min_y) / resolution))

        if global_w <= 0 or global_h <= 0 or global_w > 10000 or global_h > 10000:
            return

        # 2. Allocate canvas initialized to -1 (Unknown)
        canvas = np.full((global_h, global_w), -1, dtype=np.int8)

        # 3. Fuse each local map onto the global canvas
        for robot_map in prepared_maps:
            robot_map_width = robot_map.width
            robot_map_height = robot_map.height
            robot_map_data = robot_map.grid_data
            cos_yaw = robot_map.cos_yaw
            sin_yaw = robot_map.sin_yaw
            spawn_offset_x = robot_map.spawn_offset_x
            spawn_offset_y = robot_map.spawn_offset_y
            robot_origin_x = robot_map.origin_x
            robot_origin_y = robot_map.origin_y

            if abs(robot_map.spawn_yaw_offset) < 1e-4:
                # Fast path: pure translation (all robots facing same heading)
                # 1. World position of the robot's map origin (in meters)
                robot_origin_world_x = robot_origin_x + spawn_offset_x
                robot_origin_world_y = robot_origin_y + spawn_offset_y

                # 2. Canvas column and row where this robot's map begins
                canvas_col_origin = int(round((robot_origin_world_x - min_x) / resolution))
                canvas_row_origin = int(round((robot_origin_world_y - min_y) / resolution))

                # 3. Canvas window boundaries (clipped to keep within poster edges)
                canvas_col_start = max(0, canvas_col_origin)
                canvas_row_start = max(0, canvas_row_origin)
                canvas_col_end = min(global_w, canvas_col_origin + robot_map_width)
                canvas_row_end = min(global_h, canvas_row_origin + robot_map_height)

                # 4. Corresponding window boundaries on the robot's local map
                robot_col_start = canvas_col_start - canvas_col_origin
                robot_row_start = canvas_row_start - canvas_row_origin
                robot_col_end = robot_col_start + (canvas_col_end - canvas_col_start)
                robot_row_end = robot_row_start + (canvas_row_end - canvas_row_start)

                if canvas_col_end > canvas_col_start and canvas_row_end > canvas_row_start:
                    # 5. Extract matching patches from the global canvas and robot map
                    canvas_patch = canvas[canvas_row_start:canvas_row_end, canvas_col_start:canvas_col_end]
                    robot_patch = robot_map_data[robot_row_start:robot_row_end, robot_col_start:robot_col_end]

                    # 6. Fusion rule ("Once cleared, it stays cleared"):
                    # a) Free floor (0) marks floor and clears any old obstacles
                    is_free_floor = (robot_patch == 0)
                    canvas_patch[is_free_floor] = 0

                    # b) Obstacles (>0) are drawn on unknown/existing walls,
                    #    but FORBIDDEN from overwriting confirmed free floor (0)
                    is_obstacle = (robot_patch > 0)
                    can_draw_obstacle = is_obstacle & (canvas_patch != 0)
                    canvas_patch[can_draw_obstacle] = robot_patch[can_draw_obstacle]

                    # 7. Paste the updated patch back onto the master canvas
                    canvas[canvas_row_start:canvas_row_end, canvas_col_start:canvas_col_end] = canvas_patch
            else:
                # Rotated map placement (general affine sampling)
                y_indices, x_indices = np.where(robot_map_data >= 0)
                if len(x_indices) == 0:
                    continue

                local_x = robot_origin_x + x_indices * resolution
                local_y = robot_origin_y + y_indices * resolution

                world_x = (cos_yaw * local_x - sin_yaw * local_y) + spawn_offset_x
                world_y = (sin_yaw * local_x + cos_yaw * local_y) + spawn_offset_y

                u = np.round((world_x - min_x) / resolution).astype(int)
                v = np.round((world_y - min_y) / resolution).astype(int)

                valid = (u >= 0) & (u < global_w) & (v >= 0) & (v < global_h)
                u = u[valid]
                v = v[valid]
                vals = robot_map_data[y_indices[valid], x_indices[valid]]

                # Confirmed free floor clears obstacles and cannot be overwritten by stale obstacles
                current_vals = canvas[v, u]
                is_free = (vals == 0)
                is_obs = (vals > 0) & (current_vals != 0)
                new_vals = np.where(is_free, 0, np.where(is_obs, vals, current_vals))
                canvas[v, u] = new_vals

        # 4. Construct and publish the fused OccupancyGrid message
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'
        merged_msg.info.map_load_time = merged_msg.header.stamp
        merged_msg.info.resolution = resolution
        merged_msg.info.width = global_w
        merged_msg.info.height = global_h
        merged_msg.info.origin.position.x = float(min_x)
        merged_msg.info.origin.position.y = float(min_y)
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0

        merged_msg.data = canvas.flatten().tolist()
        self.merged_pub.publish(merged_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

