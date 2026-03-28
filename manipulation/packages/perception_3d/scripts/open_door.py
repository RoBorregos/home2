#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from frida_interfaces.action import MoveToPose, MoveJoints
from frida_interfaces.srv import DetectDoor
from frida_constants.manipulation_constants import (
    MOVE_TO_POSE_ACTION_SERVER,
    MOVE_JOINTS_ACTION_SERVER,
    GRIPPER_SET_STATE_SERVICE,
)
from frida_constants.xarm_configurations import XARM_CONFIGURATIONS
import math
import time


GRASP_MAX_RETRIES = 3
GRIPPER_CLOSED_THRESHOLD = 0.80

DETECT_DOOR_SERVICE = '/vision/detect_door'


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class DoorOpener(Node):
    def __init__(self):
        super().__init__('door_opener')

        # Action clients
        self.move_to_pose_client = ActionClient(
            self, MoveToPose, MOVE_TO_POSE_ACTION_SERVER
        )
        self.move_joints_client = ActionClient(
            self, MoveJoints, MOVE_JOINTS_ACTION_SERVER
        )

        # Gripper client
        self.gripper_client = self.create_client(SetBool, GRIPPER_SET_STATE_SERVICE)

        # Vision service client
        self.detect_door_client = self.create_client(DetectDoor, DETECT_DOOR_SERVICE)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/door_markers', 10)

        # Joint states for grasp verification
        self.drive_joint_position = None
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )

        # Detection results
        self.handle_pos = None
        self.axis_pos = None
        self.axis_seen = False

        # J6 wrist rotation for handle grasp (degrees)
        self.grasp_j6 = 135.0

    def _joint_states_cb(self, msg):
        if 'drive_joint' in msg.name:
            idx = msg.name.index('drive_joint')
            self.drive_joint_position = msg.position[idx]

    # ── Vision service call ───────────────────────────────────────────

    def detect_door(self) -> bool:
        """Call the vision service to detect handle and axis positions."""
        self.get_logger().info('Waiting for door detection service...')
        if not self.detect_door_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Door detection service not available')
            return False

        request = DetectDoor.Request()

        self.get_logger().info('Calling door detection service...')
        future = self.detect_door_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Door detection failed')
            return False

        if result.handle_detected:
            self.handle_pos = result.handle_position
            self.get_logger().info(
                f'Handle at ({self.handle_pos.x:.3f}, {self.handle_pos.y:.3f}, {self.handle_pos.z:.3f})'
            )

        if result.axis_detected:
            self.axis_pos = result.axis_position
            self.axis_seen = True
            self.get_logger().info(
                f'Axis at ({self.axis_pos.x:.3f}, {self.axis_pos.y:.3f}, {self.axis_pos.z:.3f})'
            )
        else:
            self.get_logger().info('Axis NOT detected — will PUSH door after opening handle.')

        return True

    # ── Visualization ─────────────────────────────────────────────────

    def publish_markers(self):
        markers = MarkerArray()

        if self.handle_pos is not None:
            m = Marker()
            m.header.frame_id = 'link_base'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'door_handle'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = self.handle_pos
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color.r = 1.0
            m.color.a = 1.0
            markers.markers.append(m)

        if self.axis_pos is not None:
            m2 = Marker()
            m2.header.frame_id = 'link_base'
            m2.header.stamp = self.get_clock().now().to_msg()
            m2.ns = 'door_axis'
            m2.id = 1
            m2.type = Marker.CYLINDER
            m2.action = Marker.ADD
            m2.pose.position = self.axis_pos
            m2.pose.orientation.w = 1.0
            m2.scale.x = m2.scale.y = 0.03
            m2.scale.z = 0.15
            m2.color.b = 1.0
            m2.color.a = 1.0
            markers.markers.append(m2)

        if markers.markers:
            self.marker_pub.publish(markers)

    # ── Arm movement ──────────────────────────────────────────────────

    def send_pose_goal(self, pose: Pose, phase_name: str, is_cartesian: bool = False,
                       velocity: float = 0.3, tolerance_orientation: float = 0.0) -> bool:
        self.get_logger().info(f'--- {phase_name} ---')
        self.get_logger().info(
            f'  Target: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        )
        self.move_to_pose_client.wait_for_server()

        goal_msg = MoveToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'link_base'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        goal_msg.pose = ps
        goal_msg.target_link = 'link_eef'
        goal_msg.velocity = velocity
        goal_msg.acceleration = velocity
        goal_msg.planning_time = 2.0
        goal_msg.planning_attempts = 5

        if tolerance_orientation > 0.0:
            goal_msg.tolerance_orientation = tolerance_orientation

        if is_cartesian:
            goal_msg.planner_id = 'cartesian'

        future = self.move_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{phase_name} REJECTED')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or not result.result.success:
            self.get_logger().error(f'{phase_name} FAILED (planning or execution error)')
            return False
        self.get_logger().info(f'{phase_name} DONE')
        return True

    def set_gripper(self, open_gripper: bool):
        if not self.gripper_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Gripper service not available')
            return
        req = SetBool.Request()
        req.data = open_gripper
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        state = 'OPEN' if open_gripper else 'CLOSED'
        self.get_logger().info(f'Gripper: {state}')

    def move_to_named_position(self, name: str, velocity: float = 0.3) -> bool:
        config = XARM_CONFIGURATIONS[name]
        joint_names = list(config["joints"].keys())
        joint_vals = list(config["joints"].values())
        if config.get("degrees", False):
            joint_vals = [v * math.pi / 180.0 for v in joint_vals]
        return self._send_joint_goal(joint_names, joint_vals, f'named:{name}', velocity)

    def move_joints_raw(self, joint_values_deg: list, velocity: float = 0.3, label: str = '') -> bool:
        joint_names = [f'joint{i+1}' for i in range(len(joint_values_deg))]
        joint_vals = [v * math.pi / 180.0 for v in joint_values_deg]
        return self._send_joint_goal(joint_names, joint_vals, label or 'joints_raw', velocity)

    def _send_joint_goal(self, joint_names, joint_vals, label, velocity) -> bool:
        goal = MoveJoints.Goal()
        goal.joint_names = joint_names
        goal.joint_positions = joint_vals
        goal.velocity = velocity

        self.get_logger().info(f'Moving joints: {label}')
        self.move_joints_client.wait_for_server(timeout_sec=5.0)
        future = self.move_joints_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Joint goal {label} rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result is None or not result.result.success:
            self.get_logger().error(f'Joint goal {label} FAILED')
            return False
        self.get_logger().info(f'Joint goal {label} DONE')
        return True

    def verify_grasp(self) -> bool:
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.2)
        if self.drive_joint_position is None:
            self.get_logger().warn('No drive_joint data — assuming grasp OK')
            return True
        grasped = self.drive_joint_position < GRIPPER_CLOSED_THRESHOLD
        self.get_logger().info(
            f'drive_joint={self.drive_joint_position:.3f}, '
            f'threshold={GRIPPER_CLOSED_THRESHOLD}, '
            f'grasp={"OK" if grasped else "FAILED"}'
        )
        return grasped

    def attempt_grasp(self, orientation) -> bool:
        for attempt in range(1, GRASP_MAX_RETRIES + 1):
            h = self.handle_pos
            self.get_logger().info(f'=== GRASP ATTEMPT {attempt}/{GRASP_MAX_RETRIES} ===')

            self.set_gripper(True)
            time.sleep(0.3)

            # Pre-grasp: 15cm behind handle (generous orientation tolerance for reachability)
            pose_pre = Pose()
            pose_pre.position = Point(x=h.x - 0.15, y=h.y, z=h.z)
            pose_pre.orientation = orientation
            if not self.send_pose_goal(pose_pre, 'PRE-GRASP (approach)',
                                       tolerance_orientation=1.0):
                continue

            # Set J6 to grasp angle before inserting
            if not self.move_joints_raw(
                [self.grasp_j6], velocity=0.2, label='SET J6 FOR GRASP'
            ):
                self.get_logger().warn('J6 adjustment failed, continuing anyway')

            # Grasp: move to handle (cartesian, tight tolerance)
            pose_grasp = Pose()
            pose_grasp.position = Point(x=h.x, y=h.y, z=h.z)
            pose_grasp.orientation = orientation
            if not self.send_pose_goal(pose_grasp, 'GRASP (insert)', is_cartesian=True, velocity=0.15):
                continue

            self.set_gripper(False)

            if self.verify_grasp():
                self.get_logger().info('Grasp verified successfully!')
                return True

            self.get_logger().warn(f'Grasp failed on attempt {attempt}, re-detecting...')
            self.set_gripper(True)
            time.sleep(0.3)
            self.move_to_named_position('handler_stare')

            # Re-detect via vision service
            if self.detect_door():
                self.get_logger().info('Handle re-detected, will retry.')
            else:
                self.get_logger().warn('Re-detection failed, using last known position.')

        self.get_logger().error('All grasp attempts failed!')
        return False

    def compute_approach_orientation(self):
        """Compute EEF orientation to approach the handle horizontally.

        The orientation is derived from the handle position so the gripper
        points toward the handle with the wrist (J6) rotated for a proper grip.
        roll=-π/2 gives a horizontal forward EEF on the xarm6.
        yaw aligns the approach direction toward the handle in the XY plane.
        """
        h = self.handle_pos
        yaw = math.atan2(h.y, h.x)
        roll = -math.pi / 2
        return quaternion_from_euler(roll, 0.0, yaw)

    # ── Main sequence ─────────────────────────────────────────────────

    def execute_sequence(self):
        time.sleep(1.0)

        # 0. Move to detection position
        self.get_logger().info('=== MOVING TO FRONT_LOW_STARE ===')
        if not self.move_to_named_position('front_low_stare'):
            self.get_logger().error('Failed to move to front_low_stare position')
            return

        # 1. Detect handle + axis via vision service
        self.get_logger().info('=== DETECTING DOOR HANDLE & AXIS ===')
        if not self.detect_door():
            self.get_logger().error('Cannot proceed without handle detection')
            return

        self.publish_markers()

        orientation = self.compute_approach_orientation()

        # 2. Grasp handle with retry logic
        if not self.attempt_grasp(orientation):
            self.get_logger().error('Could not grasp handle after retries, aborting.')
            return

        h = self.handle_pos

        # 3. Push down: lower handle 8cm
        pose_down = Pose()
        pose_down.position = Point(x=h.x, y=h.y, z=h.z - 0.08)
        pose_down.orientation = orientation
        if not self.send_pose_goal(pose_down, 'PUSH DOWN (lower handle)',
                                   is_cartesian=True, velocity=0.15):
            return

        # 4. Open door: PULL toward axis if seen, otherwise PULL then PUSH
        if self.axis_seen:
            self.get_logger().info('=== PULLING DOOR TOWARD AXIS ===')
            pull_dir_y = 0.10 if self.axis_pos.y > h.y else -0.10
            pose_pull = Pose()
            pose_pull.position = Point(x=h.x - 0.25, y=h.y + pull_dir_y, z=h.z - 0.08)
            pose_pull.orientation = orientation
            if not self.send_pose_goal(pose_pull, 'PULL OPEN (toward axis)',
                                       is_cartesian=True, velocity=0.2):
                return

            self.set_gripper(True)
        else:
            self.get_logger().info('=== NO AXIS: PULL UNLATCH THEN PUSH ===')
            pose_pull = Pose()
            pose_pull.position = Point(x=h.x - 0.15, y=h.y, z=h.z - 0.08)
            pose_pull.orientation = orientation
            if not self.send_pose_goal(pose_pull, 'PULL (unlatch)',
                                       is_cartesian=True, velocity=0.2):
                return

            self.set_gripper(True)
            time.sleep(0.3)

            pose_push = Pose()
            pose_push.position = Point(x=h.x + 0.20, y=h.y, z=h.z)
            pose_push.orientation = orientation
            if not self.send_pose_goal(pose_push, 'PUSH DOOR OPEN',
                                       is_cartesian=True, velocity=0.25):
                return

        self.get_logger().info('=== DOOR OPENING COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = DoorOpener()
    node.execute_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()