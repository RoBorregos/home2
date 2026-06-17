#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from frida_constants.manipulation_constants import (
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_VELOCITY,
    PICK_ACCELERATION,
    PICK_PLANNER,
    ATTACH_COLLISION_OBJECT_SERVICE,
    REMOVE_COLLISION_OBJECT_SERVICE,
    GET_COLLISION_OBJECTS_SERVICE,
    PICK_OBJECT_NAMESPACE,
    EEF_LINK_NAME,
    EEF_CONTACT_LINKS,
    PICK_MOTION_ACTION_SERVER,
    SAFETY_HEIGHT,
    CUTLERY_NAMES,
    GRASP_LINK_FRAME,
    GRIPPER_SET_STATE_SERVICE,
    GO_TO_HAND_ACTION_SERVER,
    ESTOP_TOPIC,
    RIM_NAMES,
    RIM_PRE_GRASP_HEIGHT,
    RIM_DESCENT_SPEED,
    RIM_DESCENT_DISTANCE,
    PEAK_NAMES,
    PEAK_PRE_GRASP_HEIGHT,
    XARM_ROBOT_STATES_TOPIC,
)
from frida_interfaces.srv import (
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)
from frida_interfaces.action import PickMotion, MoveToPose, GoToHand
from frida_interfaces.msg import PickResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import copy
import numpy as np
from tf_transformations import quaternion_from_euler
import tf2_ros
from transforms3d.quaternions import quat2mat
from frida_motion_planning.utils.tf_utils import transform_point
from frida_motion_planning.utils.service_utils import (
    close_gripper,
    open_gripper,
)
import time

from std_srvs.srv import Empty

# Force-guarded descent imports
from sensor_msgs.msg import JointState
from xarm_msgs.srv import MoveVelocity, SetInt16
from xarm_msgs.msg import RobotMsg

# =============================================================================
# Force-guarded descent constants
# Units: speeds in mm/s (xArm SDK convention)
# =============================================================================
CUTLERY_DESCENT_SPEED = 20.0  # mm/s downward (= 2 cm/s)
CUTLERY_EFFORT_THRESHOLD = (
    6.5  # N - effort delta to detect contact (raised from 3.0 to avoid false positives)
)
CUTLERY_DESCENT_TIMEOUT = 10.0  # s - max descent before giving up
CUTLERY_PRE_GRASP_HEIGHT = (
    0.15  # m - pre-grasp height above target (MoveIt uses meters)
)
CUTLERY_EFFORT_GRACE_PERIOD = 0.5  # s - ignore effort readings for this long after velocity starts (transient spike from mode switch)
CUTLERY_POST_CONTACT_RETRACT = 0.002  # m - retract upward after contact to relieve Z pressure before closing gripper

# Mode switching timing
MODE_SWITCH_SETTLE_TIME = 1.0  # s - wait after entering mode 5
MODE1_RECOVERY_TIME = 3.0  # s - wait after restoring mode 1 for traj controller
MODE1_RETRY_ATTEMPTS = 3  # retries for restoring mode 1

# Closed-loop fixed-distance descent constants
DESCENT_TIMEOUT_FACTOR = 2.5


class PickMotionServer(Node):
    def __init__(self):
        super().__init__("pick_server")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("ee_link_offset", -0.125)
        self.ee_link_offset = self.get_parameter("ee_link_offset").value
        self.get_logger().info(f"End-effector link offset: {self.ee_link_offset} m")

        self.declare_parameter("ee_tip_offset", -0.17)
        self.ee_tip_offset = self.get_parameter("ee_tip_offset").value
        self.get_logger().info(f"End-effector tip offset: {self.ee_tip_offset} m")

        self.declare_parameter("rim_tip_offset", -0.12)
        self.rim_tip_offset = self.get_parameter("rim_tip_offset").value
        self.get_logger().info(f"Rim tip offset: {self.rim_tip_offset} m")

        self.get_logger().info(f"Pick Velocity: {PICK_VELOCITY} m/s")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            PickMotion,
            PICK_MOTION_ACTION_SERVER,
            self.execute_callback,
            callback_group=self.callback_group,
        )

        self._go_to_hand_server = ActionServer(
            self,
            GoToHand,
            GO_TO_HAND_ACTION_SERVER,
            self.execute_go_to_hand_callback,
            callback_group=self.callback_group,
        )

        self._move_to_pose_action_client = ActionClient(
            self,
            MoveToPose,
            MOVE_TO_POSE_ACTION_SERVER,
        )

        self._attach_collision_object_client = self.create_client(
            AttachCollisionObject,
            ATTACH_COLLISION_OBJECT_SERVICE,
        )

        self._get_collision_objects_client = self.create_client(
            GetCollisionObjects,
            GET_COLLISION_OBJECTS_SERVICE,
        )

        self._remove_collision_object_client = self.create_client(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
        )

        self._gripper_set_state_client = self.create_client(
            SetBool,
            GRIPPER_SET_STATE_SERVICE,
        )

        self._clear_octomap_client = self.create_client(
            Empty,
            "/clear_octomap",
        )

        # --- Force-guarded descent service clients ---
        self._vc_set_cartesian_velocity_client = self.create_client(
            MoveVelocity, "/xarm/vc_set_cartesian_velocity"
        )
        self._set_mode_client = self.create_client(SetInt16, "/xarm/set_mode")
        self._set_state_client = self.create_client(SetInt16, "/xarm/set_state")

        # Joint effort monitoring
        self._latest_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # TCP position feedback for closed-loop descent
        self._latest_robot_state = None
        self.create_subscription(
            RobotMsg, XARM_ROBOT_STATES_TOPIC, self._robot_state_cb, 10
        )

        self._estop = False
        self.create_subscription(
            Bool,
            ESTOP_TOPIC,
            lambda msg: setattr(self, "_estop", msg.data),
            10,
        )

        self._move_to_pose_action_client.wait_for_server()
        self.get_logger().info("Pick Action Server has been started")

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def _robot_state_cb(self, msg: RobotMsg):
        self._latest_robot_state = msg

    def _get_tcp_z(self):
        """Return current TCP Z in meters (base frame), or None if no state received yet."""
        if self._latest_robot_state is None:
            return None
        return self._latest_robot_state.pose[2] / 1000.0

    def _clear_octomap(self, settle: float = 0.3):
        """Clear the octomap and wait briefly for the planning scene to update."""
        if self._clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self._clear_octomap_client.call_async(Empty.Request())
        time.sleep(settle)

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing pick goal...")

        feedback = PickMotion.Feedback()
        result = PickMotion.Result()
        try:
            result.success, result.pick_result = self.pick(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            goal_handle.succeed()
            result.success = False
            return result

    async def execute_go_to_hand_callback(self, goal_handle):
        self.get_logger().info("Executing go to hand goal...")

        base_point = copy.deepcopy(goal_handle.request.point)
        self.get_logger().info(
            f"Hand point received in frame '{base_point.header.frame_id}': "
            f"({base_point.point.x:.3f}, {base_point.point.y:.3f}, {base_point.point.z:.3f})"
        )
        # Transform to base frame if necessary
        if base_point.header.frame_id != "base_link":
            success, base_point = transform_point(
                base_point, "base_link", self.tf_buffer
            )
            if not success:
                self.get_logger().error(
                    f"Failed to transform hand point from '{goal_handle.request.point.header.frame_id}' to 'base_link'. "
                    "Check that TF is available between these frames."
                )
                result = GoToHand.Result()
                result.success = False
                goal_handle.succeed()
                return result
            self.get_logger().info(
                f"Transformed to base_link: ({base_point.point.x:.3f}, {base_point.point.y:.3f}, {base_point.point.z:.3f})"
            )
        # quaternion position
        qx, qy, qz, qw = quaternion_from_euler(-np.pi / 2, 0, 0)
        quat = [qx, qy, qz, qw]

        # quat2mat expects scalar-first [w, x, y, z]
        rotation_matrix = quat2mat([qw, qx, qy, qz])
        # Approach axis is the gripper's local Z (column 2), same convention as pick()
        approach_axis = rotation_matrix[:, 2]

        base_position = (
            np.array([base_point.point.x, base_point.point.y, base_point.point.z])
            + approach_axis * self.ee_link_offset
        )

        hand_offset = goal_handle.request.hand_offset
        test_angles = [0, 180, 200, 220, 240, 270]

        result = GoToHand.Result()

        try:
            for angle in test_angles:
                pose = PoseStamped()
                pose.header.frame_id = base_point.header.frame_id

                pose.pose.position.x = base_position[0] + hand_offset * np.cos(
                    np.radians(angle)
                )
                pose.pose.position.y = base_position[1] + hand_offset * np.sin(
                    np.radians(angle)
                )
                pose.pose.position.z = base_position[2]

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                move_result, action_result = self.move_to_pose(
                    pose=pose,
                    tolerance_position=0.01,
                    tolerance_orientation=0.1,
                )

                if action_result.result.success:
                    break

            if action_result.result.success:
                self.get_logger().info("Go to hand pose reached")
                result.success = True
            else:
                self.get_logger().error("Failed to reach go to hand pose")
                result.success = False

            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Go to hand failed: {str(e)}")
            goal_handle.succeed()
            result.success = False
            return result

    def pick(self, goal_handle, feedback):
        self.get_logger().info(
            f"Trying to pick up object: {goal_handle.request.object_name}"
        )
        pick_result = PickResult()
        grasping_poses = goal_handle.request.grasping_poses

        is_flat = goal_handle.request.object_name.lower() in CUTLERY_NAMES
        is_rim = goal_handle.request.object_name.lower() in RIM_NAMES
        is_peak = goal_handle.request.object_name.lower() in PEAK_NAMES

        if is_flat:
            num_grasping_alternatives = 6
            grasping_alternative_distance = -0.005
        elif is_rim or is_peak:
            # Each grasping_pose is already a distinct candidate;
            # no extra z-offset alternatives needed.
            num_grasping_alternatives = 1
            grasping_alternative_distance = 0.0
        else:
            num_grasping_alternatives = 2
            grasping_alternative_distance = -0.025

        self.save_collision_objects()

        for i, pose in enumerate(grasping_poses):
            if self._estop:
                return False, pick_result
            for j in range(num_grasping_alternatives):
                if self._estop:
                    return False, pick_result
                ee_link_pose = copy.deepcopy(pose)
                # Rim: offset by full tip distance so fingers straddle the wall without descending too far.
                offset_distance = self.rim_tip_offset if is_rim else self.ee_link_offset
                offset_distance += j * grasping_alternative_distance

                quat = [
                    ee_link_pose.pose.orientation.w,
                    ee_link_pose.pose.orientation.x,
                    ee_link_pose.pose.orientation.y,
                    ee_link_pose.pose.orientation.z,
                ]
                rotation_matrix = quat2mat(quat)
                z_axis = rotation_matrix[:, 2]

                new_position = (
                    np.array(
                        [
                            ee_link_pose.pose.position.x,
                            ee_link_pose.pose.position.y,
                            ee_link_pose.pose.position.z,
                        ]
                    )
                    + z_axis * offset_distance
                )
                ee_link_pose.pose.position.x = new_position[0]
                ee_link_pose.pose.position.y = new_position[1]
                ee_link_pose.pose.position.z = new_position[2]

                if is_flat:
                    self.get_logger().info(
                        f"[Cutlery] Force-guarded pick flow for alternative {j}"
                    )

                    # Open gripper
                    self.get_logger().info("[Cutlery] Opening gripper...")
                    self._gripper_set_state_client.wait_for_service(timeout_sec=2.0)
                    open_gripper(self._gripper_set_state_client)
                    time.sleep(0.5)

                    # Pre-grasp: 15cm above
                    pre_grasp_pose = copy.deepcopy(ee_link_pose)
                    pre_grasp_pose.pose.position.z += CUTLERY_PRE_GRASP_HEIGHT

                    self.get_logger().info(
                        f"[Cutlery] Pre-grasp Z={pre_grasp_pose.pose.position.z:.4f} "
                        f"(target Z={ee_link_pose.pose.position.z:.4f})"
                    )

                    self._clear_octomap()

                    pre_handler, pre_result = self.move_to_pose(
                        pre_grasp_pose, velocity=0.3
                    )

                    if not pre_result.result.success:
                        self.get_logger().warn(
                            f"[Cutlery] Pre-grasp failed for alternative {j}, trying next"
                        )
                        continue

                    self.get_logger().info("[Cutlery] Pre-grasp reached")

                    # Clear octomap
                    self.get_logger().info("[Cutlery] Clearing octomap...")
                    self._clear_octomap()

                    # Force-guarded descent
                    self.get_logger().info(
                        "[Cutlery] Starting force-guarded descent..."
                    )
                    contact = self.force_guarded_descent()

                    if contact:
                        # Retract slightly to relieve Z pressure so gripper can close fully
                        self.get_logger().info(
                            f"[Cutlery] Contact detected! Retracting {CUTLERY_POST_CONTACT_RETRACT * 1000:.1f}mm before closing gripper..."
                        )
                        retract_pose = copy.deepcopy(pre_grasp_pose)
                        retract_pose.pose.position.z = (
                            ee_link_pose.pose.position.z + CUTLERY_POST_CONTACT_RETRACT
                        )
                        self.move_to_pose(retract_pose, velocity=0.1)
                        time.sleep(0.5)

                        self.get_logger().info("[Cutlery] Closing gripper...")
                        close_gripper(self._gripper_set_state_client)
                        time.sleep(1.5)
                        self.get_logger().info("[Cutlery] Gripper closed")

                        # Lift
                        self.get_logger().info("[Cutlery] Lifting...")
                        self.move_to_pose(pre_grasp_pose, velocity=0.2)

                        pick_result.pick_pose = ee_link_pose
                        pick_result.grasp_score = goal_handle.request.grasping_scores[i]
                        pick_result.object_pick_height = 0.0
                        pick_result.object_height = 0.0

                        self.get_logger().info("[Cutlery] Pick complete!")
                        return True, pick_result
                    else:
                        self.get_logger().warn(
                            f"[Cutlery] No contact for alternative {j}, trying next"
                        )
                        continue

                elif is_rim:
                    self.get_logger().info(
                        f"[Rim] Fixed-distance pick flow for pose {i}"
                    )

                    # Open gripper
                    self.get_logger().info("[Rim] Opening gripper...")
                    self._gripper_set_state_client.wait_for_service(timeout_sec=2.0)
                    open_gripper(self._gripper_set_state_client)
                    time.sleep(0.5)

                    # Pre-grasp above the rim (MoveIt)
                    pre_grasp_pose = copy.deepcopy(ee_link_pose)
                    pre_grasp_pose.pose.position.z += RIM_PRE_GRASP_HEIGHT

                    self.get_logger().info(
                        f"[Rim] Pre-grasp Z={pre_grasp_pose.pose.position.z:.4f} "
                        f"(rim Z={ee_link_pose.pose.position.z:.4f})"
                    )

                    self._clear_octomap()

                    pre_handler, pre_result = self.move_to_pose(
                        pre_grasp_pose, velocity=0.3
                    )

                    if not pre_result.result.success:
                        self.get_logger().warn(
                            f"[Rim] Pre-grasp failed for pose {i}, trying next"
                        )
                        continue

                    self.get_logger().info("[Rim] Pre-grasp reached")

                    # Clear octomap again before the descent phase.
                    self.get_logger().info("[Rim] Clearing octomap...")
                    self._clear_octomap()

                    # Fixed-distance descent (xArm cartesian velocity)
                    self.get_logger().info(
                        f"[Rim] Descending a fixed {RIM_DESCENT_DISTANCE * 1000:.0f}mm..."
                    )
                    descended = self.fixed_distance_descent(RIM_DESCENT_DISTANCE)

                    if not descended:
                        self.get_logger().warn(
                            f"[Rim] Descent failed for pose {i}, trying next"
                        )
                        continue

                    # Close gripper on the rim
                    self.get_logger().info("[Rim] Closing gripper...")
                    close_gripper(self._gripper_set_state_client)
                    time.sleep(1.5)
                    self.get_logger().info("[Rim] Gripper closed")

                    self.get_logger().info("[Rim] Pick complete!")
                    return True, pick_result

                elif is_peak:
                    self.get_logger().info(
                        f"[Peak] Fixed-distance pick flow for pose {i}"
                    )

                    # Open gripper
                    self.get_logger().info("[Peak] Opening gripper...")
                    self._gripper_set_state_client.wait_for_service(timeout_sec=2.0)
                    open_gripper(self._gripper_set_state_client)
                    time.sleep(0.5)

                    # Pre-grasp above the content peak (MoveIt)
                    pre_grasp_pose = copy.deepcopy(ee_link_pose)
                    pre_grasp_pose.pose.position.z += PEAK_PRE_GRASP_HEIGHT

                    self.get_logger().info(
                        f"[Peak] Pre-grasp Z={pre_grasp_pose.pose.position.z:.4f} "
                        f"(content Z={ee_link_pose.pose.position.z:.4f})"
                    )

                    self._clear_octomap()

                    pre_handler, pre_result = self.move_to_pose(
                        pre_grasp_pose, velocity=0.3
                    )

                    if not pre_result.result.success:
                        self.get_logger().warn(
                            f"[Peak] Pre-grasp failed for pose {i}, trying next"
                        )
                        continue

                    self.get_logger().info("[Peak] Pre-grasp reached")

                    # Clear octomap again before the descent phase.
                    self.get_logger().info("[Peak] Clearing octomap...")
                    self._clear_octomap()

                    # Fixed-distance close-loop descent (reuses rim mechanism):
                    # drives the pre-grasp offset plus penetration into the pile.
                    self.get_logger().info(
                        f"[Peak] Descending a fixed {PEAK_PRE_GRASP_HEIGHT * 1000:.0f}mm..."
                    )
                    descended = self.fixed_distance_descent(PEAK_PRE_GRASP_HEIGHT)

                    if not descended:
                        self.get_logger().warn(
                            f"[Peak] Descent failed for pose {i}, trying next"
                        )
                        continue

                    # Close gripper on the content
                    self.get_logger().info("[Peak] Closing gripper...")
                    close_gripper(self._gripper_set_state_client)
                    time.sleep(1.5)
                    self.get_logger().info("[Peak] Gripper closed")

                    self.get_logger().info("[Peak] Pick complete!")
                    return True, pick_result

                else:
                    grasp_pose_handler, grasp_pose_result = self.move_to_pose(
                        ee_link_pose
                    )

                print(f"Grasp Pose {i} result: {grasp_pose_result}")
                if grasp_pose_result.result.success:
                    self.get_logger().info("Grasp pose reached")
                    result, lowest_obj, highest_obj = self.attach_pick_object()

                    self.get_logger().info("Closing gripper")
                    close_gripper(self._gripper_set_state_client)
                    time.sleep(1.5)
                    self.get_logger().info("Gripper closed")

                    if result:
                        self.get_logger().info("Object attached")
                        pick_result.pick_pose = ee_link_pose
                        pick_result.grasp_score = goal_handle.request.grasping_scores[i]

                        if lowest_obj is not None:
                            pick_result.object_pick_height = (
                                self.calculate_object_pick_height(
                                    lowest_obj, ee_link_pose
                                )
                            )
                        else:
                            pick_result.object_pick_height = 0.0

                        if lowest_obj is not None and highest_obj is not None:
                            pick_result.object_height = self.calculate_object_height(
                                lowest_obj, highest_obj
                            )
                        else:
                            pick_result.object_height = 0.0
                    else:
                        self.get_logger().error("Failed to attach object")

                    return True, pick_result

        self.get_logger().error("Failed to reach any grasp pose")
        return False, pick_result

    # ==================================================================
    # Force-Guarded Descent
    # ==================================================================

    def force_guarded_descent(self) -> bool:
        """
        Descend in -Z using xArm mode 5 (cartesian velocity control).
        Single velocity command (persists until zero-vel), monitor effort for contact.
        Grace period ignores transient effort spikes from mode switching.
        """
        self.get_logger().info("[ForceGuard] Starting force-guarded descent")

        # --- 1. Effort baseline ---
        if self._latest_joint_state is None:
            self.get_logger().error("[ForceGuard] No joint_states received!")
            return False

        baselines = []
        for _ in range(10):
            if self._latest_joint_state is not None:
                baselines.append(list(self._latest_joint_state.effort))
            time.sleep(0.02)

        if len(baselines) < 5:
            self.get_logger().error("[ForceGuard] Not enough baseline readings")
            return False

        effort_baseline = np.median(baselines, axis=0)
        self.get_logger().info(
            f"[ForceGuard] Baseline: {[f'{e:.2f}' for e in effort_baseline]}"
        )

        # --- 2. Transition: mode 1 -> 0 -> 5 ---
        self.get_logger().info("[ForceGuard] Mode transition 1 -> 0 -> 5...")

        if not self._set_xarm_mode(0):
            self.get_logger().error("[ForceGuard] Failed to set mode 0")
            return False
        time.sleep(0.5)

        if not self._set_xarm_mode(5):
            self.get_logger().error("[ForceGuard] Failed to set mode 5")
            self._restore_mode1()
            return False

        self.get_logger().info(
            f"[ForceGuard] Waiting {MODE_SWITCH_SETTLE_TIME}s for mode 5 settle..."
        )
        time.sleep(MODE_SWITCH_SETTLE_TIME)

        # --- Re-capture baseline AFTER mode switch to avoid transient offsets ---
        baselines_post = []
        for _ in range(10):
            if self._latest_joint_state is not None:
                baselines_post.append(list(self._latest_joint_state.effort))
            time.sleep(0.02)
        if len(baselines_post) >= 5:
            effort_baseline = np.median(baselines_post, axis=0)
            self.get_logger().info(
                f"[ForceGuard] Post-mode-switch baseline: {[f'{e:.2f}' for e in effort_baseline]}"
            )

        # --- 3. Send ONE velocity command ---
        contact_detected = False
        start_time = time.time()

        try:
            if not self._vc_set_cartesian_velocity_client.wait_for_service(
                timeout_sec=2.0
            ):
                self.get_logger().error(
                    "[ForceGuard] vc_set_cartesian_velocity not available"
                )
                self._restore_mode1()
                return False

            vel_req = MoveVelocity.Request()
            vel_req.speeds = [0.0, 0.0, -CUTLERY_DESCENT_SPEED, 0.0, 0.0, 0.0]
            vel_req.is_tool_coord = False
            vel_req.duration = 0.0

            vel_future = self._vc_set_cartesian_velocity_client.call_async(vel_req)
            self.wait_for_future(vel_future)

            if vel_future.result() is None:
                self.get_logger().error("[ForceGuard] Velocity command failed")
                self._restore_mode1()
                return False

            self.get_logger().info(
                f"[ForceGuard] Descending at {CUTLERY_DESCENT_SPEED:.1f} mm/s "
                f"(threshold={CUTLERY_EFFORT_THRESHOLD}N, grace={CUTLERY_EFFORT_GRACE_PERIOD}s)"
            )

            # --- 4. Monitor effort ---
            while (time.time() - start_time) < CUTLERY_DESCENT_TIMEOUT:
                time.sleep(0.02)  # 50Hz

                elapsed = time.time() - start_time

                if self._latest_joint_state is None:
                    continue

                current_effort = np.array(self._latest_joint_state.effort)
                effort_delta = np.abs(current_effort - effort_baseline)
                max_delta = float(np.max(effort_delta))
                max_joint = int(np.argmax(effort_delta))

                # Skip during grace period (transient spikes from mode switch / motion start)
                if elapsed < CUTLERY_EFFORT_GRACE_PERIOD:
                    continue

                if max_delta > CUTLERY_EFFORT_THRESHOLD:
                    distance_mm = elapsed * CUTLERY_DESCENT_SPEED
                    self.get_logger().info(
                        f"[ForceGuard] CONTACT! Joint {max_joint + 1} "
                        f"delta={max_delta:.2f}N (threshold={CUTLERY_EFFORT_THRESHOLD}N) "
                        f"after {elapsed:.2f}s (~{distance_mm:.1f}mm descended)"
                    )
                    contact_detected = True
                    break

            if not contact_detected:
                elapsed = time.time() - start_time
                self.get_logger().warn(
                    f"[ForceGuard] Timeout after {elapsed:.1f}s "
                    f"(~{elapsed * CUTLERY_DESCENT_SPEED:.1f}mm descended)"
                )

        except Exception as e:
            self.get_logger().error(f"[ForceGuard] Descent error: {e}")

        # --- 5. Stop ---
        self._stop_cartesian_velocity()

        # --- 6. Restore mode 1 ---
        self._restore_mode1()

        return contact_detected

    def fixed_distance_descent(self, distance_m: float) -> bool:
        """
        Descend a fixed distance in -Z using xArm mode 5 (cartesian velocity control).
        Closed-loop on /xarm/robot_states TCP Z.
        """
        self.get_logger().info(
            f"[FixedDescent] Descending {distance_m * 1000:.0f}mm at "
            f"{RIM_DESCENT_SPEED:.1f} mm/s"
        )

        # --- Read start position before any mode switch ---
        wait_start = time.time()
        while self._get_tcp_z() is None:
            if time.time() - wait_start > 2.0:
                self.get_logger().error(
                    "[FixedDescent] No robot_states received after 2s — cannot verify descent"
                )
                return False
            time.sleep(0.05)
        start_z = self._get_tcp_z()
        target_z = start_z - distance_m
        self.get_logger().info(
            f"[FixedDescent] start_z={start_z * 1000:.1f}mm  "
            f"target_z={target_z * 1000:.1f}mm"
        )

        # --- Transition: mode 1 -> 0 -> 5 ---
        if not self._set_xarm_mode(0):
            self.get_logger().error("[FixedDescent] Failed to set mode 0")
            return False
        time.sleep(0.5)

        if not self._set_xarm_mode(5):
            self.get_logger().error("[FixedDescent] Failed to set mode 5")
            self._restore_mode1()
            return False

        self.get_logger().info(
            f"[FixedDescent] Waiting {MODE_SWITCH_SETTLE_TIME}s for mode 5 settle..."
        )
        time.sleep(MODE_SWITCH_SETTLE_TIME)

        reached = False
        try:
            if not self._vc_set_cartesian_velocity_client.wait_for_service(
                timeout_sec=2.0
            ):
                self.get_logger().error(
                    "[FixedDescent] vc_set_cartesian_velocity not available"
                )
                return False

            vel_req = MoveVelocity.Request()
            vel_req.speeds = [0.0, 0.0, -RIM_DESCENT_SPEED, 0.0, 0.0, 0.0]
            vel_req.is_tool_coord = False
            vel_req.duration = 0.0

            # Initial velocity command
            vel_future = self._vc_set_cartesian_velocity_client.call_async(vel_req)
            self.wait_for_future(vel_future)
            if vel_future.result() is None:
                self.get_logger().error(
                    "[FixedDescent] Initial velocity command failed"
                )
                return False

            expected_time = (distance_m * 1000.0) / RIM_DESCENT_SPEED
            timeout = expected_time * DESCENT_TIMEOUT_FACTOR

            loop_start = time.time()

            while True:
                time.sleep(0.02)  # 50 Hz
                now = time.time()
                elapsed = now - loop_start

                # Timeout guard
                if elapsed > timeout:
                    self.get_logger().warn(
                        f"[FixedDescent] Timeout after {elapsed:.1f}s "
                        f"(limit={timeout:.1f}s)"
                    )
                    break

                # E-stop
                if self._estop:
                    self.get_logger().warn("[FixedDescent] E-stop during descent")
                    break

                cur_z = self._get_tcp_z()
                if cur_z is None:
                    continue

                descended = start_z - cur_z

                # Reached target?
                if descended >= distance_m:
                    self.get_logger().info(
                        f"[FixedDescent] Reached! descended={descended * 1000:.1f}mm "
                        f"(target={distance_m * 1000:.0f}mm)"
                    )
                    reached = True
                    break

        except Exception as e:
            self.get_logger().error(f"[FixedDescent] Descent error: {e}")

        finally:
            self._stop_cartesian_velocity()
            self._restore_mode1()

        return reached

    def _set_xarm_mode(self, mode: int) -> bool:
        try:
            if not self._set_mode_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("[ForceGuard] set_mode service unavailable")
                return False

            mode_req = SetInt16.Request()
            mode_req.data = mode
            mode_future = self._set_mode_client.call_async(mode_req)
            self.wait_for_future(mode_future)
            if mode_future.result() is None:
                self.get_logger().error(f"[ForceGuard] set_mode({mode}) returned None")
                return False

            self.get_logger().info(f"[ForceGuard] Mode -> {mode}")
            time.sleep(0.3)

            if not self._set_state_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("[ForceGuard] set_state service unavailable")
                return False

            state_req = SetInt16.Request()
            state_req.data = 0
            state_future = self._set_state_client.call_async(state_req)
            self.wait_for_future(state_future)
            if state_future.result() is None:
                self.get_logger().error("[ForceGuard] set_state(0) returned None")
                return False

            self.get_logger().info(f"[ForceGuard] Mode {mode}, State 0 OK")
            return True

        except Exception as e:
            self.get_logger().error(f"[ForceGuard] _set_xarm_mode({mode}) error: {e}")
            return False

    def _stop_cartesian_velocity(self):
        try:
            stop_req = MoveVelocity.Request()
            stop_req.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            stop_req.is_tool_coord = False
            stop_req.duration = 0.0
            if self._vc_set_cartesian_velocity_client.wait_for_service(timeout_sec=1.0):
                stop_future = self._vc_set_cartesian_velocity_client.call_async(
                    stop_req
                )
                self.wait_for_future(stop_future)
            self.get_logger().info("[ForceGuard] Velocity zeroed")
        except Exception as e:
            self.get_logger().error(f"[ForceGuard] Stop error: {e}")

    def _restore_mode1(self):
        self.get_logger().info("[ForceGuard] Restoring mode 1...")

        for attempt in range(MODE1_RETRY_ATTEMPTS):
            if not self._set_xarm_mode(0):
                self.get_logger().warn(
                    f"[ForceGuard] Mode 0 failed (attempt {attempt + 1})"
                )
                time.sleep(1.0)
                continue

            time.sleep(0.5)

            if not self._set_xarm_mode(1):
                self.get_logger().warn(
                    f"[ForceGuard] Mode 1 failed (attempt {attempt + 1})"
                )
                time.sleep(1.0)
                continue

            self.get_logger().info(
                f"[ForceGuard] Waiting {MODE1_RECOVERY_TIME}s for traj controller..."
            )
            time.sleep(MODE1_RECOVERY_TIME)

            self.get_logger().info(
                f"[ForceGuard] Mode 1 restored (attempt {attempt + 1})"
            )
            return

        self.get_logger().error("[ForceGuard] CRITICAL: Could not restore mode 1!")

    # ==================================================================
    # Existing Methods (unchanged)
    # ==================================================================

    def move_to_pose(
        self,
        pose,
        tolerance_position=0.005,
        tolerance_orientation=0.02,
        velocity=PICK_VELOCITY,
    ):
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = float(velocity)
        request.acceleration = float(PICK_ACCELERATION)
        request.planner_id = PICK_PLANNER
        request.target_link = GRASP_LINK_FRAME
        request.tolerance_position = tolerance_position
        request.tolerance_orientation = tolerance_orientation
        future = self._move_to_pose_action_client.send_goal_async(request)
        self.wait_for_future(future)
        action_result = future.result().get_result()
        return future.result(), action_result

    def wait_for_future(self, future):
        if future is None:
            self.get_logger().error("Service call failed: future is None")
            return False
        while not future.done():
            pass
        return future

    def save_collision_objects(self):
        self.collision_objects = self.get_collision_objects()

    def attach_pick_object(self):
        obj_lowest = None
        obj_highest = None
        for obj in self.collision_objects:
            if PICK_OBJECT_NAMESPACE in obj.id:
                request = AttachCollisionObject.Request()
                request.id = obj.id
                if obj_lowest is None:
                    obj_lowest = obj
                else:
                    if obj.pose.pose.position.z < obj_lowest.pose.pose.position.z:
                        obj_lowest = obj
                if obj_highest is None:
                    obj_highest = obj
                else:
                    if obj.pose.pose.position.z > obj_highest.pose.pose.position.z:
                        obj_highest = obj
                request.attached_link = EEF_LINK_NAME
                request.touch_links = EEF_CONTACT_LINKS
                request.detach = False
                self._attach_collision_object_client.wait_for_service()
                future = self._attach_collision_object_client.call_async(request)
                self.wait_for_future(future)
        return True, obj_lowest, obj_highest

    def get_collision_objects(self):
        request = GetCollisionObjects.Request()
        future = self._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)
        return future.result().collision_objects

    def remove_collision_object(self, id):
        request = RemoveCollisionObject.Request()
        request.id = id
        self._remove_collision_object_client.wait_for_service()
        future = self._remove_collision_object_client.call_async(request)
        self.wait_for_future(future)
        return future.result().success

    def calculate_object_pick_height(self, obj, pose):
        if obj.pose.header.frame_id != pose.header.frame_id:
            self.get_logger().error(
                "Object and pose frames do not match, cannot calculate height"
            )
            return 0.0
        obj_z = obj.pose.pose.position.z
        obj_radius = obj.dimensions.x
        grasp_height = pose.pose.position.z
        height = grasp_height - (obj_z - obj_radius) + SAFETY_HEIGHT
        self.get_logger().info(f"Object pick height: {height}")
        return height

    def calculate_object_height(self, obj_lowest, obj_highest):
        if obj_lowest.pose.header.frame_id != obj_highest.pose.header.frame_id:
            self.get_logger().error(
                "Object and pose frames do not match, cannot calculate height"
            )
            return 0.0
        obj_lowest_z = obj_lowest.pose.pose.position.z
        obj_highest_z = obj_highest.pose.pose.position.z
        obj_radius = obj_lowest.dimensions.x
        height = (obj_highest_z + obj_radius) - (obj_lowest_z - obj_radius)
        self.get_logger().info(f"Object height: {height}")
        return height


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PickMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
