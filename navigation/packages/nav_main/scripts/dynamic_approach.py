#!/usr/bin/env python3
"""
Dynamic Approach Node - Sensor-based final approach to furniture/objects.

After Nav2 brings the robot near a target, this node takes over and moves
the robot forward slowly using direct sensor feedback (lidar + depth camera)
until it reaches a safe minimum distance from the obstacle.

Works independently of SLAM/costmaps - purely reactive control based on
what's directly in front of the robot in base_link frame.

Key behaviors:
- Floating table: lidar sees nothing underneath, depth sees table top above
  → robot drives further in, getting the arm closer
- Shelf/solid furniture: lidar detects front surface → stops before collision
- Combines lidar (precise 2D at scan height) with depth camera (3D volume)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from frida_interfaces.action import Approach

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def read_points_from_cloud(cloud_msg, fields=('x', 'y', 'z')):
    """Extract XYZ points from a PointCloud2 message using numpy."""
    field_map = {f.name: f.offset for f in cloud_msg.fields}
    point_step = cloud_msg.point_step
    raw = bytes(cloud_msg.data)
    n_points = cloud_msg.width * cloud_msg.height

    if n_points == 0 or len(raw) < point_step:
        return np.zeros((0, len(fields)), dtype=np.float32)

    # Reshape raw bytes into per-point chunks and extract float32 at offsets
    data = np.frombuffer(raw, dtype=np.uint8).reshape(n_points, point_step)
    points = np.zeros((n_points, len(fields)), dtype=np.float32)
    for i, name in enumerate(fields):
        if name in field_map:
            off = field_map[name]
            points[:, i] = data[:, off:off+4].view(np.float32).flatten()

    # Filter NaN/Inf
    valid = np.isfinite(points).all(axis=1)
    return points[valid]


class DynamicApproachNode(Node):
    def __init__(self):
        super().__init__('dynamic_approach')

        self.action_cb_group = ReentrantCallbackGroup()
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()

        # Action server
        self.action_server = ActionServer(
            self, Approach, '/navigation/approach',
            self.approach_callback,
            callback_group=self.action_cb_group
        )

        # Cmd vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Lidar subscriber
        self.latest_scan = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.sensor_cb_group
        )

        # Depth point cloud subscriber
        self.latest_cloud = None
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/point_cloud_nav', self.cloud_callback, sensor_qos,
            callback_group=self.sensor_cb_group
        )

        # TF for transforming point cloud to base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.is_approaching = False

        self.get_logger().info('DynamicApproach action server ready on /navigation/approach')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def get_lidar_min_distance(self, cone_angle):
        """Get minimum distance from lidar in a forward cone.
        Lidar is in base_link frame, forward is angle 0."""
        scan = self.latest_scan
        if scan is None:
            return float('inf')

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * scan.angle_increment + scan.angle_min

        # Filter: within cone, within valid range, and finite
        in_cone = np.abs(angles) <= cone_angle
        valid = (ranges > scan.range_min) & (ranges < scan.range_max) & np.isfinite(ranges)
        mask = in_cone & valid

        if not mask.any():
            return float('inf')

        return float(np.min(ranges[mask]))

    def get_depth_min_distance(self, cone_angle, min_height, max_height):
        """Get minimum distance from depth point cloud in a forward cone,
        filtered by height range. Points are transformed to base_link frame."""
        cloud = self.latest_cloud
        if cloud is None:
            return float('inf')

        # Transform point cloud to base_link if needed
        if cloud.header.frame_id != 'base_link':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', cloud.header.frame_id,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                cloud = do_transform_cloud(cloud, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(f'TF transform failed: {e}')
                return float('inf')

        points = read_points_from_cloud(cloud, ('x', 'y', 'z'))
        if len(points) == 0:
            return float('inf')

        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # Filter: forward only (x > 0), within height range
        # and within the cone angle (|atan2(y, x)| < cone_angle)
        forward = x > 0.05  # At least 5cm forward
        height_ok = (z >= min_height) & (z <= max_height)
        angles = np.abs(np.arctan2(y, x))
        in_cone = angles < cone_angle

        mask = forward & height_ok & in_cone
        if not mask.any():
            return float('inf')

        distances = np.sqrt(x[mask] ** 2 + y[mask] ** 2)
        return float(np.min(distances))

    def stop_robot(self):
        """Send zero velocity."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def approach_callback(self, goal_handle):
        """Execute the dynamic approach."""
        goal = goal_handle.request
        min_dist = goal.min_distance
        approach_speed = goal.approach_speed
        max_distance = goal.max_distance
        depth_min_h = goal.depth_min_height
        depth_max_h = goal.depth_max_height
        cone_angle = goal.cone_angle

        self.get_logger().info(
            f'Approach started: min_dist={min_dist:.2f}m, '
            f'speed={approach_speed:.2f}m/s, cone={math.degrees(cone_angle):.0f}deg')

        self.is_approaching = True
        feedback = Approach.Feedback()
        result = Approach.Result()

        # Control loop at 10Hz
        rate_period = 0.1
        slowdown_distance = min_dist * 3.0  # Start slowing at 3x min distance
        stall_count = 0
        max_stall = 30  # 3 seconds of no progress

        prev_distance = float('inf')

        try:
            while rclpy.ok() and self.is_approaching:
                # Check if cancelled
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Approach cancelled'
                    self.is_approaching = False
                    return result

                # Get distances from both sensors
                lidar_dist = self.get_lidar_min_distance(cone_angle)
                depth_dist = self.get_depth_min_distance(
                    cone_angle, depth_min_h, depth_max_h)

                # Use the minimum of both sensors
                current_dist = min(lidar_dist, depth_dist)

                # Determine which sensor is dominant for logging
                sensor = 'lidar' if lidar_dist <= depth_dist else 'depth'

                # Handle no sensor data
                if current_dist == float('inf'):
                    # No obstacles detected - check if we've been searching too long
                    stall_count += 1
                    if stall_count > max_stall:
                        self.stop_robot()
                        result.success = False
                        result.final_distance = float('inf')
                        result.message = 'No obstacles detected in approach cone'
                        goal_handle.succeed()
                        self.is_approaching = False
                        return result

                    # Move forward slowly hoping to see something
                    twist = Twist()
                    twist.linear.x = approach_speed * 0.5
                    self.cmd_vel_pub.publish(twist)

                    feedback.current_distance = -1.0
                    feedback.status = 'searching'
                    goal_handle.publish_feedback(feedback)
                    time.sleep(rate_period)
                    continue

                stall_count = 0

                # Check if too far to approach
                if current_dist > max_distance:
                    self.stop_robot()
                    result.success = False
                    result.final_distance = current_dist
                    result.message = f'Too far from obstacle ({current_dist:.2f}m > {max_distance:.2f}m)'
                    goal_handle.succeed()
                    self.is_approaching = False
                    return result

                # Check if we've reached the target
                if current_dist <= min_dist:
                    self.stop_robot()
                    self.get_logger().info(
                        f'\033[92mApproach complete!\033[0m '
                        f'Distance: {current_dist:.3f}m ({sensor})')
                    result.success = True
                    result.final_distance = current_dist
                    result.message = f'Reached target distance ({sensor}: {current_dist:.3f}m)'
                    goal_handle.succeed()
                    self.is_approaching = False
                    return result

                # Calculate approach velocity with proportional slowdown
                if current_dist < slowdown_distance:
                    # Proportional: full speed at slowdown_distance, near-zero at min_dist
                    t = (current_dist - min_dist) / (slowdown_distance - min_dist)
                    t = max(0.1, min(1.0, t))  # Clamp between 0.1 and 1.0
                    speed = approach_speed * t
                else:
                    speed = approach_speed

                # Ensure minimum creep speed
                speed = max(speed, 0.02)

                # Publish velocity
                twist = Twist()
                twist.linear.x = speed
                self.cmd_vel_pub.publish(twist)

                # Determine status
                if current_dist < slowdown_distance:
                    status = 'slowing'
                else:
                    status = 'approaching'

                feedback.current_distance = current_dist
                feedback.status = status
                goal_handle.publish_feedback(feedback)

                self.get_logger().debug(
                    f'{status}: dist={current_dist:.3f}m ({sensor}), '
                    f'speed={speed:.3f}m/s')

                # Check for stall (robot not getting closer)
                if abs(current_dist - prev_distance) < 0.002:
                    stall_count += 1
                    if stall_count > max_stall:
                        self.stop_robot()
                        result.success = True
                        result.final_distance = current_dist
                        result.message = f'Stalled at {current_dist:.3f}m (close enough)'
                        goal_handle.succeed()
                        self.is_approaching = False
                        return result
                else:
                    stall_count = 0
                prev_distance = current_dist

                time.sleep(rate_period)

        except Exception as e:
            self.stop_robot()
            self.get_logger().error(f'Approach error: {e}')
            result.success = False
            result.message = f'Error: {str(e)}'
            goal_handle.succeed()
            self.is_approaching = False
            return result

        self.stop_robot()
        result.success = False
        result.message = 'Approach ended unexpectedly'
        goal_handle.succeed()
        self.is_approaching = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DynamicApproachNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
