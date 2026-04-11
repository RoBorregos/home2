#!/usr/bin/env python3

from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class AdaptiveGoalPublisher(Node):
    """Publishes adjusted goals when the original goal is blocked by obstacles."""

    def __init__(self):
        super().__init__("adaptive_goal_publisher")

        self.declare_parameter("obstacle_threshold", 90)
        self.declare_parameter("free_threshold", 50)
        self.declare_parameter("search_radius", 2.0)
        self.declare_parameter("check_rate", 2.0)
        self.declare_parameter("safety_margin_cells", 3)

        self.obstacle_threshold = self.get_parameter("obstacle_threshold").value
        self.free_threshold = self.get_parameter("free_threshold").value
        self.search_radius = self.get_parameter("search_radius").value
        check_rate = self.get_parameter("check_rate").value
        self.safety_margin = self.get_parameter("safety_margin_cells").value
        
        self.declare_parameter("approach_min_dist", 0.5)
        self.declare_parameter("approach_max_dist", 1.2)
        self.declare_parameter("lock_proximity_threshold", 1.0)  # Distancia máxima a la meta original

        self.approach_min_dist = self.get_parameter("approach_min_dist").value
        self.approach_max_dist = self.get_parameter("approach_max_dist").value
        self.lock_threshold = self.get_parameter("lock_proximity_threshold").value

        self.person_position = None
        self.original_goal = None
        self.current_costmap = None
        self.costmap_info = None
        self.is_goal_blocked = False
        self.last_published_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/adaptive_nav/original_goal",
            self.goal_callback,
            10,
        )

        self.tracking_sub = self.create_subscription(
            PointStamped,
            "/vision/tracking_results",
            self.tracking_callback,
            10,
        )

        self.updated_goal_pub = self.create_publisher(
            PoseStamped, "/goal_update", 10
        )
        self.goal_blocked_pub = self.create_publisher(
            Bool, "/adaptive_nav/goal_blocked", 10
        )

        self.check_timer = self.create_timer(1.0 / check_rate, self.check_goal_validity)

        self.get_logger().info(
            f"AdaptiveGoalPublisher ready "
            f"(obstacle_thresh={self.obstacle_threshold}, "
            f"search_radius={self.search_radius}m, "
            f"safety_margin={self.safety_margin} cells)"
        )

    def costmap_callback(self, msg: OccupancyGrid):
        """Store the latest costmap as a numpy array."""
        self.current_costmap = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.costmap_info = msg.info

    def tracking_callback(self, msg: PointStamped):
        """Update the live person's position in the map frame with proximity gating."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.5)
            )
            transformed = do_transform_point(msg, transform)
            new_x = transformed.point.x
            new_y = transformed.point.y

            # 1. Chequeo contra la meta original (ancla de la mesa)
            if self.original_goal:
                gx = self.original_goal.pose.position.x
                gy = self.original_goal.pose.position.y
                dist_to_goal = math.sqrt((new_x - gx)**2 + (new_y - gy)**2)
                
                if dist_to_goal > self.lock_threshold:
                    # La persona está muy lejos de la mesa objetivo, ignorar
                    return

            # 2. Hysteresis/Lock: Evitar saltar entre personas en la misma mesa
            if self.person_position:
                prev_x, prev_y = self.person_position
                dist_to_prev = math.sqrt((new_x - prev_x)**2 + (new_y - prev_y)**2)
                
                # Si el salto es demasiado grande (> 1m), probablemente es otra persona
                # y nos quedamos con la que ya teníamos
                if dist_to_prev > 1.2:
                    return

            self.person_position = (new_x, new_y)
            
        except TransformException as e:
            self.get_logger().warn(f"Could not transform tracking point to map: {e}")

    def goal_callback(self, msg: PoseStamped):
        """Receive a new original goal from the task manager."""
        self.original_goal = msg
        self.is_goal_blocked = False
        self.last_published_goal = None
        self.person_position = None  # Reset lock when goal changes
        self.get_logger().info(
            f"New original goal (Table Anchor): ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def get_robot_position(self):
        """Get the robot's current position in the map frame via TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException as e:
            self.get_logger().warn(f"Could not get robot position: {e}")
            return None

    def world_to_costmap(self, world_x: float, world_y: float):
        """Convert world (map frame) coordinates to costmap grid indices."""
        if self.costmap_info is None:
            return None, None

        mx = int(
            (world_x - self.costmap_info.origin.position.x)
            / self.costmap_info.resolution
        )
        my = int(
            (world_y - self.costmap_info.origin.position.y)
            / self.costmap_info.resolution
        )

        if 0 <= mx < self.costmap_info.width and 0 <= my < self.costmap_info.height:
            return mx, my
        return None, None

    def costmap_to_world(self, mx: int, my: int):
        """Convert costmap grid indices to world (map frame) coordinates."""
        wx = (
            self.costmap_info.origin.position.x
            + (mx + 0.5) * self.costmap_info.resolution
        )
        wy = (
            self.costmap_info.origin.position.y
            + (my + 0.5) * self.costmap_info.resolution
        )
        return wx, wy

    def get_cost_at(self, mx: int, my: int) -> int:
        """Get costmap cost at grid position. Returns -1 if out of bounds."""
        if 0 <= mx < self.costmap_info.width and 0 <= my < self.costmap_info.height:
            return int(self.current_costmap[my][mx])
        return -1

    def is_position_blocked(self, world_x: float, world_y: float) -> bool:
        """Check if a world position is blocked."""
        mx, my = self.world_to_costmap(world_x, world_y)
        if mx is None:
            return True
        cost = self.get_cost_at(mx, my)
        return cost < 0 or cost >= self.obstacle_threshold

    def raycast_find_approach_point(self, robot_x, robot_y, goal_x, goal_y):
        """
        Cast a ray from robot to goal on the costmap grid.
        Walk cell by cell along the line. Return the last FREE cell before
        hitting an obstacle.

        This guarantees the point is:
          - On the robot's side of the obstacle
          - Reachable from the robot (along the approach direction)
          - As close as possible to the goal

        Uses Bresenham's line algorithm for pixel-perfect ray walking.

        Returns (world_x, world_y) or None if no free cell found.
        """
        r_mx, r_my = self.world_to_costmap(robot_x, robot_y)
        g_mx, g_my = self.world_to_costmap(goal_x, goal_y)

        if r_mx is None or g_mx is None:
            return None

        cells = self._bresenham_line(r_mx, r_my, g_mx, g_my)

        last_free = None
        for mx, my in cells:
            cost = self.get_cost_at(mx, my)
            if cost < 0:
                continue  # Out of bounds

            if cost < self.free_threshold:
                last_free = (mx, my)
            elif cost >= self.obstacle_threshold:
                if last_free is not None:
                    return self._apply_safety_margin(last_free, cells)
                break  # No free cell found before obstacle

        return None  # Entire ray is blocked

    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm — returns list of (x,y) grid cells along the line."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    def _apply_safety_margin(self, free_cell, ray_cells):
        """
        Back off a few cells from the obstacle edge along the ray direction.
        Returns world coordinates of the safe approach point.
        """
        try:
            idx = ray_cells.index(free_cell)
        except ValueError:
            wx, wy = self.costmap_to_world(free_cell[0], free_cell[1])
            return (wx, wy)

        safe_idx = max(0, idx - self.safety_margin)

        for i in range(safe_idx, -1, -1):
            mx, my = ray_cells[i]
            cost = self.get_cost_at(mx, my)
            if 0 <= cost < self.free_threshold:
                wx, wy = self.costmap_to_world(mx, my)
                return (wx, wy)

        # Fallback to the original free cell
        wx, wy = self.costmap_to_world(free_cell[0], free_cell[1])
        return (wx, wy)

    def bfs_find_nearest_free(self, goal_x, goal_y):
        goal_mx, goal_my = self.world_to_costmap(goal_x, goal_y)
        if goal_mx is None:
            return None

        max_cells = int(self.search_radius / self.costmap_info.resolution)
        width = self.costmap_info.width
        height = self.costmap_info.height

        queue = deque()
        queue.append((goal_mx, goal_my, 0))
        visited = set()
        visited.add((goal_mx, goal_my))

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while queue:
            cx, cy, dist = queue.popleft()

            if dist > max_cells:
                break

            cost = self.get_cost_at(cx, cy)
            if cost >= 0 and cost < self.free_threshold and dist > 0:
                wx, wy = self.costmap_to_world(cx, cy)
                return (wx, wy)

            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited and 0 <= nx < width and 0 <= ny < height:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        return None

    def radial_sampling_approach(self, robot_x, robot_y, goal_x, goal_y):
        """
        Sample candidate points in a ring around the detected person's live position.
        Picks the free one closest to the robot.
        Returns (wx, wy, quaternion) so the robot forces orientation to the person.
        """
        center_x = self.person_position[0] if self.person_position is not None else goal_x
        center_y = self.person_position[1] if self.person_position is not None else goal_y

        N = 36
        candidates = []
        for i in range(N):
            angle = i * (2 * math.pi / N)
            for dist in np.linspace(self.approach_min_dist, self.approach_max_dist, 5):
                cand_x = center_x + dist * math.cos(angle)
                cand_y = center_y + dist * math.sin(angle)
                
                mx, my = self.world_to_costmap(cand_x, cand_y)
                if mx is not None:
                    cost = self.get_cost_at(mx, my)
                    if 0 <= cost < self.free_threshold:
                        rb_dist = math.hypot(robot_x - cand_x, robot_y - cand_y)
                        candidates.append((rb_dist, cand_x, cand_y))
        
        if not candidates:
            return None
        
        # sort by distance from robot
        candidates.sort(key=lambda x: x[0])
        best_x = candidates[0][1]
        best_y = candidates[0][2]

        # calculate quaternion to face the person
        yaw = math.atan2(center_y - best_y, center_x - best_x)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = qz
        q.w = qw
        
        return (best_x, best_y, q)

    def find_best_approach_point(self, goal_x, goal_y):
        robot_pos = self.get_robot_position()
        if robot_pos is not None:
            # 1. Radial Sampling Strategy
            result = self.radial_sampling_approach(
                robot_pos[0], robot_pos[1], goal_x, goal_y
            )
            if result is not None:
                self.get_logger().info(
                    f"Radial sampling found approach point: ({result[0]:.2f}, {result[1]:.2f})"
                )
                return result

            # 2. Raycast fallback
            result = self.raycast_find_approach_point(
                robot_pos[0], robot_pos[1], goal_x, goal_y
            )
            if result is not None:
                self.get_logger().info(
                    f"Raycast found approach point: ({result[0]:.2f}, {result[1]:.2f})"
                )
                return (result[0], result[1], None)
            else:
                self.get_logger().warn(
                    "Raycast failed, trying BFS"
                )

        # 3. BFS fallback
        result = self.bfs_find_nearest_free(goal_x, goal_y)
        if result is not None:
            self.get_logger().info(
                f"BFS found nearest free point: ({result[0]:.2f}, {result[1]:.2f})"
            )
            return (result[0], result[1], None)
        return None

    def check_goal_validity(self):
        if self.original_goal is None or self.current_costmap is None:
            return

        goal_x = self.original_goal.pose.position.x
        goal_y = self.original_goal.pose.position.y

        blocked = self.is_position_blocked(goal_x, goal_y)

        blocked_msg = Bool()
        blocked_msg.data = blocked
        self.goal_blocked_pub.publish(blocked_msg)

        if blocked:
            if not self.is_goal_blocked:
                self.get_logger().warn(
                    f"Goal ({goal_x:.2f}, {goal_y:.2f}) is blocked "
                    "Searching for nearest free point"
                )
            self.is_goal_blocked = True

            result = self.find_best_approach_point(goal_x, goal_y)

            if result is not None:
                new_x, new_y, new_q = result
                dist = math.hypot(new_x - goal_x, new_y - goal_y)

                if self.last_published_goal is None or math.hypot(
                    new_x - self.last_published_goal[0],
                    new_y - self.last_published_goal[1],
                ) > self.costmap_info.resolution * 2:
                    self.get_logger().info(
                        f"Adjusted goal  ({new_x:.2f}, {new_y:.2f}), "
                        f"{dist:.2f}m from original"
                    )

                    updated = PoseStamped()
                    updated.header.frame_id = "map"
                    updated.header.stamp = self.get_clock().now().to_msg()
                    updated.pose.position.x = new_x
                    updated.pose.position.y = new_y
                    updated.pose.position.z = 0.0
                    updated.pose.orientation = new_q if new_q is not None else self.original_goal.pose.orientation

                    self.updated_goal_pub.publish(updated)
                    self.last_published_goal = (new_x, new_y)
            else:
                self.get_logger().error(
                    f"No free point within {self.search_radius}m of goal"
                )
        else:
            if self.is_goal_blocked:
                self.get_logger().info(
                    "reverting to original goal"
                )
                self.updated_goal_pub.publish(self.original_goal)
                self.last_published_goal = None
            self.is_goal_blocked = False


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
