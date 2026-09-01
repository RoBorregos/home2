"""RobotArm: everything the robot can do, in one place.

The pipelines (`pipelines/pick.py`, `place.py`, `pour.py`) talk only to this
class and never touch a ROS client, which is what keeps them readable and what
lets them be unit-tested against a fake arm with no ROS graph.
"""

import time
from collections import deque
from contextlib import contextmanager
from typing import Optional

import numpy as np
from frida_constants.manipulation_constants import (
    ADD_COLLISION_OBJECT_SERVICE,
    ATTACH_COLLISION_OBJECT_SERVICE,
    EEF_CONTACT_LINKS,
    EEF_LINK_NAME,
    ESTOP_TOPIC,
    GET_COLLISION_OBJECTS_SERVICE,
    GET_JOINT_SERVICE,
    GRASP_LINK_FRAME,
    GRIPPER_SET_STATE_SERVICE,
    MOVE_JOINTS_ACTION_SERVER,
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_ACCELERATION,
    PICK_OBJECT_NAMESPACE,
    PICK_PLANNER,
    PICK_VELOCITY,
    REMOVE_COLLISION_OBJECT_SERVICE,
    SAFETY_HEIGHT,
    SCAN_ANGLE_HORIZONTAL,
    SCAN_ANGLE_VERTICAL,
    XARM_ROBOT_STATES_TOPIC,
)
from frida_interfaces.action import MoveJoints, MoveToPose
from frida_interfaces.msg import CollisionObject
from frida_interfaces.srv import (
    AddCollisionObjects,
    AttachCollisionObject,
    GetCollisionObjects,
    GetJoints,
    RemoveCollisionObject,
)
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.service_utils import (
    close_gripper,
    get_joint_positions,
    move_joint_positions,
    open_gripper,
)
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool
from tf2_ros import Buffer, TransformListener
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveVelocity, SetInt16
from xarm_utils.shelf_levels import get_compartment_height

from pick_and_place.pipelines.errors import PickAborted, PickHardwareError
from pick_and_place.pipelines.profiles import ForceGuardProfile
from pick_and_place.utils.grasp_utils import move_to_pregrasp_nearest_ik
from pick_and_place.utils.self_collision_utils import endpoint_self_collides

# xArm mode-machine timing. These describe the controller, not any particular
# pick behavior, so they are not part of the per-strategy profiles.
MODE_SWITCH_SETTLE_TIME = 1.0  # s - wait after entering mode 5
MODE1_RECOVERY_TIME = 3.0  # s - wait after restoring mode 1 for traj controller
MODE1_RETRY_ATTEMPTS = 3  # retries for restoring mode 1
DESCENT_TIMEOUT_FACTOR = 2.5  # closed-loop descent timeout vs. expected duration


class ContactResult:
    """Outcome of a force-guarded descent.

    ``descended_m`` is meaningful whether or not contact was detected; the
    flat strategy uses it to retract from the true contact point rather than
    a nominal target.
    """

    __slots__ = ("contact", "descended_m")

    def __init__(self, contact: bool, descended_m: float):
        self.contact = contact
        self.descended_m = descended_m

    def __repr__(self):
        return (
            f"ContactResult(contact={self.contact}, descended_m={self.descended_m:.4f})"
        )


class AttachResult:
    """Outcome of attaching the picked object to the gripper in the scene."""

    __slots__ = ("attached", "lowest_object", "highest_object")

    def __init__(self, attached: bool, lowest_object=None, highest_object=None):
        self.attached = attached
        self.lowest_object = lowest_object
        self.highest_object = highest_object


class RobotArm:
    """The robot, as the pipelines see it."""

    def __init__(self, node):
        self._node = node
        self._log = node.get_logger()
        group = node.callback_group

        # --- state fed by subscriptions ------------------------------------
        self._latest_joint_state: Optional[JointState] = None
        self._latest_robot_state: Optional[RobotMsg] = None
        self._estop = False
        self._scene_snapshot = []

        # --- observability context ------------------------------------------
        self._goal_handle = None
        self._feedback_factory = None
        self._context = ""

        # --- motion ----------------------------------------------------------
        self._move_to_pose_client = ActionClient(
            node, MoveToPose, MOVE_TO_POSE_ACTION_SERVER, callback_group=group
        )
        self._move_joints_client = ActionClient(
            node, MoveJoints, MOVE_JOINTS_ACTION_SERVER, callback_group=group
        )
        self._get_joints_client = node.create_client(
            GetJoints, GET_JOINT_SERVICE, callback_group=group
        )

        # --- gripper -----------------------------------------------------------
        self._gripper_client = node.create_client(
            SetBool, GRIPPER_SET_STATE_SERVICE, callback_group=group
        )

        # --- planning scene -------------------------------------------------------
        self._attach_client = node.create_client(
            AttachCollisionObject, ATTACH_COLLISION_OBJECT_SERVICE, callback_group=group
        )
        self._get_objects_client = node.create_client(
            GetCollisionObjects, GET_COLLISION_OBJECTS_SERVICE, callback_group=group
        )
        self._remove_object_client = node.create_client(
            RemoveCollisionObject, REMOVE_COLLISION_OBJECT_SERVICE, callback_group=group
        )
        self._add_objects_client = node.create_client(
            AddCollisionObjects, ADD_COLLISION_OBJECT_SERVICE, callback_group=group
        )
        self._clear_octomap_client = node.create_client(
            Empty, "/clear_octomap", callback_group=group
        )

        # --- MoveIt validation -------------------------------------------------------
        self._compute_ik_client = node.create_client(
            GetPositionIK, "/compute_ik", callback_group=group
        )
        self._state_validity_client = node.create_client(
            GetStateValidity, "/check_state_validity", callback_group=group
        )

        # --- xArm low level ------------------------------------------------------------
        self._cartesian_velocity_client = node.create_client(
            MoveVelocity, "/xarm/vc_set_cartesian_velocity", callback_group=group
        )
        self._set_mode_client = node.create_client(
            SetInt16, "/xarm/set_mode", callback_group=group
        )
        self._set_state_client = node.create_client(
            SetInt16, "/xarm/set_state", callback_group=group
        )

        # --- subscriptions ---------------------------------------------------------------
        node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10, callback_group=group
        )
        node.create_subscription(
            RobotMsg,
            XARM_ROBOT_STATES_TOPIC,
            self._on_robot_state,
            10,
            callback_group=group,
        )
        node.create_subscription(
            Bool, ESTOP_TOPIC, self._on_estop, 10, callback_group=group
        )

        # One TF listener for the whole node.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, node, qos=qos, spin_thread=True
        )

        # Debug views for RViz.
        self._place_pose_pub = node.create_publisher(
            PoseStamped, "/manipulator/place_ee_link_pose", 10
        )
        self._debug_point_pub = node.create_publisher(
            PointStamped, "/manipulation/debug_point", 10
        )

    def now(self):
        """Current time as a message stamp."""
        return self._node.get_clock().now().to_msg()

    def publish_place_pose(self, pose) -> None:
        """Publish the chosen place pose for RViz."""
        self._place_pose_pub.publish(pose)

    def publish_debug_point(self, point) -> None:
        """Publish a point of interest for RViz."""
        self._debug_point_pub.publish(point)

    # ==================================================================
    # Subscriptions
    # ==================================================================

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg

    def _on_robot_state(self, msg):
        self._latest_robot_state = msg

    def _on_estop(self, msg):
        self._estop = msg.data

    # ==================================================================
    # State
    # ==================================================================

    @property
    def logger(self):
        return self._log

    @property
    def estop_active(self) -> bool:
        return self._estop

    @property
    def joint_state(self) -> Optional[JointState]:
        return self._latest_joint_state

    def tcp_z(self) -> Optional[float]:
        """Current tool-centre-point Z in metres, or None if unknown."""
        if self._latest_robot_state is None:
            return None
        return self._latest_robot_state.pose[2] / 1000.0

    def tip_offset(self, param_name: str) -> float:
        """Read a tip-offset ROS parameter by name."""
        return float(self._node.get_parameter(param_name).value)

    def check_abort(self) -> None:
        """Raise PickAborted if the e-stop is active or the goal was cancelled."""
        if self._estop:
            raise PickAborted("e-stop active")
        if self._goal_handle is not None and self._goal_handle.is_cancel_requested:
            raise PickAborted("goal cancelled")

    def wait_for_future(self, future, timeout=60):
        """Wait for a future, yielding to the executor. False on timeout."""
        result = wait_for_future(future, timeout=timeout)
        if result is False:
            self._log.error(f"Future did not complete within {timeout}s")
            return False
        return result

    # ==================================================================
    # Observability
    # ==================================================================

    def bind_goal(self, goal_handle, feedback_factory=None):
        """Attach the in-flight goal, enabling cancellation and feedback."""
        self._goal_handle = goal_handle
        self._feedback_factory = feedback_factory
        self._context = ""

    def release_goal(self):
        self._goal_handle = None
        self._feedback_factory = None
        self._context = ""

    def set_context(self, context: str) -> None:
        """Label the phases that follow, so logs and feedback name the attempt."""
        self._context = context

    @contextmanager
    def phase(self, name: str):
        """Scope one phase of a task, for logging and action feedback."""
        label = f"[{self._context}] {name}" if self._context else f"[{name}]"
        self._publish_feedback(f"{self._context}/{name}" if self._context else name)
        self._log.info(f"{label}: start")
        started = time.time()
        try:
            yield
        finally:
            self._log.info(f"{label}: done in {time.time() - started:.2f}s")

    def _publish_feedback(self, state: str):
        if self._goal_handle is None or self._feedback_factory is None:
            return
        try:
            feedback = self._feedback_factory()
            feedback.execution_state = state
            self._goal_handle.publish_feedback(feedback)
        except Exception as exc:  # feedback must never break a task
            self._log.debug(f"Could not publish feedback: {exc}")

    # ==================================================================
    # Motion
    # ==================================================================

    def wait_until_ready(self):
        """Block until the motion planner is up. Called once at startup."""
        self._move_to_pose_client.wait_for_server()

    def move_to_pose(
        self,
        pose,
        velocity: Optional[float] = None,
        acceleration: Optional[float] = None,
        tolerance_position: float = 0.005,
        tolerance_orientation: float = 0.02,
        planner_id: str = PICK_PLANNER,
        target_link: str = GRASP_LINK_FRAME,
        planning_time: Optional[float] = None,
        planning_attempts: Optional[int] = None,
        constraint=None,
    ) -> bool:
        """Plan and execute straight to ``pose``. False if planning failed."""
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = float(PICK_VELOCITY if velocity is None else velocity)
        request.acceleration = float(
            PICK_ACCELERATION if acceleration is None else acceleration
        )
        request.planner_id = planner_id
        request.target_link = target_link
        request.tolerance_position = tolerance_position
        request.tolerance_orientation = tolerance_orientation
        if planning_time is not None:
            request.planning_time = float(planning_time)
        if planning_attempts is not None:
            request.planning_attempts = int(planning_attempts)
        if constraint is not None:
            request.apply_constraint = True
            request.constraint = constraint

        future = self._move_to_pose_client.send_goal_async(request)
        if not self.wait_for_future(future) or future.result() is None:
            raise PickHardwareError("motion_planning_server did not accept the goal")
        goal_handle = future.result()
        # get_result() blocks for the whole motion; the wait above only covers
        # goal acceptance, which is fast.
        return bool(goal_handle.get_result().result.success)

    def move_to_pregrasp(self, pose, velocity: float = PICK_VELOCITY) -> bool:
        """Move to a pre-grasp via the nearest IK solution.

        A free pose goal lets the planner wind joint 1 almost 360 degrees around
        the back; seeding IK with the current state keeps the motion short.
        """
        return move_to_pregrasp_nearest_ik(
            self._compute_ik_client,
            self._move_joints_client,
            pose,
            latest_joint_state=self._latest_joint_state,
            velocity=velocity,
            fallback_move_to_pose=self._move_to_pose_pair,
            logger=self._log,
        )

    def _move_to_pose_pair(self, pose, velocity=PICK_VELOCITY):
        """Adapter for move_to_pregrasp_nearest_ik, which wants (_, result)."""

        class _Wrapped:
            def __init__(self, success):
                self.result = type("R", (), {"success": success})()

        return None, _Wrapped(self.move_to_pose(pose, velocity=velocity))

    def move_to_named_position(self, name: str, velocity: float = 0.75) -> bool:
        """Move to a named joint configuration (``table_stare`` and friends)."""
        return bool(
            move_joint_positions(
                move_joints_action_client=self._move_joints_client,
                named_position=name,
                velocity=velocity,
            )
        )

    def move_joints(self, joint_positions, velocity: float = 0.2) -> bool:
        return bool(
            move_joint_positions(
                move_joints_action_client=self._move_joints_client,
                joint_positions=joint_positions,
                velocity=velocity,
            )
        )

    def get_joints(self, degrees: bool = True):
        return get_joint_positions(self._get_joints_client, degrees=degrees)

    def scan_environment(self):
        """Sweep the wrist to fill in the octomap around the workspace."""
        joints = self.get_joints(degrees=True)
        self._log.info(f"Scanning environment from: {joints}")
        joints["joints"]["joint1"] += SCAN_ANGLE_HORIZONTAL
        joints["joints"]["joint5"] -= SCAN_ANGLE_VERTICAL
        self.move_joints(joints)
        joints["joints"]["joint1"] -= SCAN_ANGLE_HORIZONTAL * 2
        self.move_joints(joints)
        joints["joints"]["joint1"] += SCAN_ANGLE_HORIZONTAL
        joints["joints"]["joint5"] += SCAN_ANGLE_VERTICAL
        self.move_joints(joints)

    def endpoint_self_collides(self, pose) -> bool:
        """True if the robot would collide with itself at ``pose``.

        World objects are ignored on purpose: the basket or pile being picked
        from would otherwise veto every candidate.
        """
        return endpoint_self_collides(
            self._compute_ik_client,
            self._state_validity_client,
            pose,
            latest_joint_state=self._latest_joint_state,
            logger=self._log,
        )

    # ==================================================================
    # Gripper
    # ==================================================================

    def open_gripper(self, settle_s: float = 0.5) -> None:
        self._gripper_client.wait_for_service(timeout_sec=2.0)
        open_gripper(self._gripper_client)
        time.sleep(settle_s)

    def close_gripper(self, settle_s: float = 1.5) -> None:
        self._gripper_client.wait_for_service(timeout_sec=2.0)
        close_gripper(self._gripper_client)
        time.sleep(settle_s)

    # ==================================================================
    # Planning scene
    # ==================================================================

    def clear_octomap(self, settle_s: float = 0.3) -> None:
        """Clear the octomap so stale voxels do not block the next plan."""
        if self._clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self._clear_octomap_client.call_async(Empty.Request())
        time.sleep(settle_s)

    def get_collision_objects(self):
        request = GetCollisionObjects.Request()
        future = self._get_objects_client.call_async(request)
        if not self.wait_for_future(future, timeout=10) or future.result() is None:
            self._log.error("get_collision_objects failed; assuming empty scene")
            return []
        return future.result().collision_objects

    def snapshot_scene(self):
        """Remember the perceived objects so a later attach can pick one."""
        self._scene_snapshot = self.get_collision_objects()
        return self._scene_snapshot

    def remove_collision_object(self, object_id: str) -> bool:
        request = RemoveCollisionObject.Request()
        request.id = object_id
        if not self._remove_object_client.wait_for_service(timeout_sec=5.0):
            self._log.error("remove_collision_object service not available")
            return False
        future = self._remove_object_client.call_async(request)
        if not self.wait_for_future(future, timeout=10) or future.result() is None:
            self._log.error(f"remove_collision_object({object_id}) failed")
            return False
        return future.result().success

    def remove_all_collision_objects(self, attached: bool = False) -> bool:
        request = RemoveCollisionObject.Request()
        request.id = "all"
        request.include_attached = attached
        if not self._remove_object_client.wait_for_service(timeout_sec=5.0):
            self._log.error("remove_collision_object service not available")
            return False
        future = self._remove_object_client.call_async(request)
        if not self.wait_for_future(future, timeout=10) or future.result() is None:
            return False
        return future.result().success

    def attach_pick_object(self) -> AttachResult:
        """Attach the lowest perceived pick object to the gripper.

        Only ONE box is attached and the rest dropped: attaching the whole
        cluster cloud (~15 boxes ahead of the gripper) over-constrained the
        return motion and made front_stare/table_stare fail.
        """
        pick_objs = [o for o in self._scene_snapshot if PICK_OBJECT_NAMESPACE in o.id]
        if not pick_objs:
            return AttachResult(attached=True)

        lowest = min(pick_objs, key=lambda o: o.pose.pose.position.z)
        highest = max(pick_objs, key=lambda o: o.pose.pose.position.z)
        for obj in pick_objs:
            if obj.id == lowest.id:
                request = AttachCollisionObject.Request()
                request.id = obj.id
                request.attached_link = EEF_LINK_NAME
                request.touch_links = EEF_CONTACT_LINKS
                request.detach = False
                self._attach_client.wait_for_service()
                self.wait_for_future(
                    self._attach_client.call_async(request), timeout=10
                )
            else:
                self.remove_collision_object(obj.id)
        return AttachResult(attached=True, lowest_object=lowest, highest_object=highest)

    def detach_pick_objects(self) -> bool:
        """Release everything attached to the gripper, after a place."""
        for obj in self.get_collision_objects():
            if PICK_OBJECT_NAMESPACE not in obj.id:
                continue
            request = AttachCollisionObject.Request()
            request.id = obj.id
            request.attached_link = EEF_LINK_NAME
            request.touch_links = EEF_CONTACT_LINKS
            request.detach = True
            if not self._attach_client.wait_for_service(timeout_sec=5.0):
                self._log.error("attach_collision_object service not available")
                continue
            self.wait_for_future(self._attach_client.call_async(request), timeout=10)
        return True

    def add_shelf_ceiling_guard(self, place_pose, table_height: float) -> None:
        """Block the place return from routing up into the compartment ceiling.

        The eye-in-hand octomap never sees it, so a slab is added explicitly.
        """
        compartment_h = get_compartment_height()
        obj = CollisionObject()
        obj.id = "shelf_ceiling_guard"
        obj.type = "box"
        obj.pose.header.frame_id = place_pose.header.frame_id or "base_link"
        obj.pose.header.stamp = self._node.get_clock().now().to_msg()
        obj.pose.pose.position.x = float(place_pose.pose.position.x) + 0.10
        obj.pose.pose.position.y = float(place_pose.pose.position.y)
        obj.pose.pose.position.z = float(table_height) + compartment_h + 0.10
        obj.pose.pose.orientation.w = 1.0
        obj.dimensions.x = 0.40
        obj.dimensions.y = 0.80
        obj.dimensions.z = 0.20

        request = AddCollisionObjects.Request()
        request.collision_objects.append(obj)
        if not self._add_objects_client.wait_for_service(timeout_sec=3.0):
            self._log.warn("add_collision_objects unavailable; no ceiling guard")
            return
        self.wait_for_future(self._add_objects_client.call_async(request), timeout=3.0)
        self._log.info("Added shelf ceiling guard collision")

    def remove_shelf_ceiling_guard(self) -> None:
        self.remove_collision_object("shelf_ceiling_guard")

    def object_pick_height(self, obj, grasp_pose) -> float:
        """Height of the grasp above the bottom of ``obj``, for placing later."""
        if obj.pose.header.frame_id != grasp_pose.header.frame_id:
            self._log.error("Object and pose frames differ, cannot calculate height")
            return 0.0
        radius = obj.dimensions.x
        height = (
            grasp_pose.pose.position.z
            - (obj.pose.pose.position.z - radius)
            + SAFETY_HEIGHT
        )
        self._log.info(f"Object pick height: {height}")
        return height

    def object_height(self, lowest, highest) -> float:
        """Overall height of the picked object, for placing later."""
        if lowest.pose.header.frame_id != highest.pose.header.frame_id:
            self._log.error("Object frames differ, cannot calculate height")
            return 0.0
        radius = lowest.dimensions.x
        height = (highest.pose.pose.position.z + radius) - (
            lowest.pose.pose.position.z - radius
        )
        self._log.info(f"Object height: {height}")
        return height

    # ==================================================================
    # xArm cartesian-velocity descents
    # ==================================================================

    @contextmanager
    def cartesian_velocity_mode(self, label: str):
        """Hold the arm in mode 5 for the duration of the block.

        Whatever happens inside -- an exception, an e-stop, an early return --
        the exit path zeroes the velocity and restores mode 1. Leaving the arm
        in mode 5 takes the trajectory controller offline, so every subsequent
        MoveIt goal would fail.
        """
        if not self._set_xarm_mode(0):
            raise PickHardwareError(f"[{label}] could not enter mode 0")
        time.sleep(0.5)

        if not self._set_xarm_mode(5):
            self._restore_mode1()
            raise PickHardwareError(f"[{label}] could not enter mode 5")

        self._log.info(f"[{label}] waiting {MODE_SWITCH_SETTLE_TIME}s for mode 5")
        time.sleep(MODE_SWITCH_SETTLE_TIME)
        try:
            yield
        finally:
            self._stop_cartesian_velocity()
            self._restore_mode1()

    def _send_cartesian_velocity(self, speed_mm_s: float, label: str) -> None:
        if not self._cartesian_velocity_client.wait_for_service(timeout_sec=2.0):
            raise PickHardwareError(f"[{label}] vc_set_cartesian_velocity unavailable")

        request = MoveVelocity.Request()
        request.speeds = [0.0, 0.0, speed_mm_s, 0.0, 0.0, 0.0]
        request.is_tool_coord = False
        request.duration = 0.0

        future = self._cartesian_velocity_client.call_async(request)
        if not self.wait_for_future(future, timeout=5) or future.result() is None:
            raise PickHardwareError(f"[{label}] velocity command failed")

    def force_guarded_descent(self, guard: ForceGuardProfile) -> ContactResult:
        """Descend in -Z until joint effort reports contact.

        Contact is detected from a sudden effort *jump* against a trailing
        reference rather than an absolute threshold: effort drifts several N
        from rest during a clean descent, so an absolute test false-trips.
        """
        label = "ForceGuard"
        self._log.info(f"[{label}] starting force-guarded descent")

        baseline = self._sample_effort_baseline(label)
        if baseline is None:
            return ContactResult(contact=False, descended_m=0.0)

        with self.cartesian_velocity_mode(label):
            # Re-sample after the mode switch: the transition itself offsets the
            # readings, and the pre-switch baseline would bias every comparison.
            post_switch = self._sample_effort_baseline(label)
            if post_switch is not None:
                baseline = post_switch

            self._send_cartesian_velocity(-guard.descent_speed, label)
            self._log.info(
                f"[{label}] descending at {guard.descent_speed:.1f} mm/s "
                f"(jump_trip={guard.jump_trip}N, ceiling={guard.hard_ceiling}N)"
            )
            contact, descended = self._monitor_for_contact(guard, baseline, label)

        return ContactResult(contact=contact, descended_m=descended)

    def _sample_effort_baseline(self, label: str, required: int = 5):
        """Median joint effort over a short window, or None if unavailable."""
        if self._latest_joint_state is None:
            self._log.error(f"[{label}] no joint_states received")
            return None

        samples = []
        for _ in range(10):
            if self._latest_joint_state is not None:
                samples.append(list(self._latest_joint_state.effort))
            time.sleep(0.02)

        if len(samples) < required:
            self._log.error(f"[{label}] not enough baseline readings")
            return None

        baseline = np.median(samples, axis=0)
        self._log.info(f"[{label}] effort baseline: {[f'{e:.2f}' for e in baseline]}")
        return baseline

    def _monitor_for_contact(self, guard: ForceGuardProfile, baseline, label: str):
        """Watch joint effort at 50 Hz. Returns (contact, metres descended)."""
        history = deque()  # (timestamp, effort) inside the jump window
        consecutive = 0
        last_log = 0.0
        start_time = time.time()

        while (time.time() - start_time) < guard.timeout:
            time.sleep(0.02)
            elapsed = time.time() - start_time

            # The mode-5 loop bypasses MoveIt, so nothing else would stop the arm
            # on an e-stop; the context manager restores mode 1 on the way out.
            if self._estop:
                self._log.warn(f"[{label}] e-stop during descent")
                raise PickAborted("e-stop during force-guarded descent")

            if self._latest_joint_state is None:
                continue

            current = np.array(self._latest_joint_state.effort)

            # Skip the motion-onset transient before detecting anything.
            if elapsed < guard.contact_settle:
                continue

            now = time.time()
            history.append((now, current))
            while len(history) > 1 and now - history[0][0] > guard.jump_window:
                history.popleft()

            descended = elapsed * guard.descent_speed / 1000.0
            abs_delta = np.abs(current - baseline)
            jump = np.abs(current - history[0][1])
            for joint in guard.ignore_joints:
                if joint < len(abs_delta):
                    abs_delta[joint] = 0.0
                    jump[joint] = 0.0

            if elapsed - last_log > 1.5:
                self._log.info(
                    f"[{label}] t={elapsed:.1f}s ~{descended * 1000:.0f}mm "
                    f"max_abs={float(np.max(abs_delta)):.2f}N "
                    f"max_jump={float(np.max(jump)):.2f}N"
                )
                last_log = elapsed

            # Real surface contact is deep; ignore trips before the minimum.
            if descended < guard.min_contact_descent:
                continue

            consecutive = consecutive + 1 if (jump > guard.jump_trip).any() else 0
            hard_stop = float(np.max(abs_delta)) > guard.hard_ceiling
            if consecutive >= guard.jump_sustain or hard_stop:
                by_jump = consecutive >= guard.jump_sustain
                metric = jump if by_jump else abs_delta
                joint = int(np.argmax(metric))
                self._log.info(
                    f"[{label}] CONTACT ({'jump' if by_jump else 'ceiling'})! "
                    f"joint {joint + 1} {float(metric[joint]):.2f}N after "
                    f"{elapsed:.2f}s (~{descended * 1000:.1f}mm descended)"
                )
                return True, descended

        elapsed = time.time() - start_time
        descended = elapsed * guard.descent_speed / 1000.0
        self._log.warn(
            f"[{label}] timeout after {elapsed:.1f}s "
            f"(~{descended * 1000:.1f}mm descended)"
        )
        return False, descended

    def fixed_distance_descent(
        self, distance_m: float, speed_mm_s: float, descend: bool = True
    ) -> bool:
        """Move a fixed Z distance under cartesian velocity, closed on TCP Z.

        ``descend=False`` drives +Z instead, reusing the same mechanism.
        """
        label = "FixedDescent"
        sign = -1.0 if descend else 1.0
        direction = "descending" if descend else "ascending"
        self._log.info(
            f"[{label}] {direction} {distance_m * 1000:.0f}mm at {speed_mm_s:.1f} mm/s"
        )

        # Read the start height before any mode switch, so a stale TCP reading
        # cannot be mistaken for progress.
        wait_start = time.time()
        while self.tcp_z() is None:
            if time.time() - wait_start > 2.0:
                self._log.error(
                    f"[{label}] no robot_states after 2s, cannot verify descent"
                )
                return False
            time.sleep(0.05)

        start_z = self.tcp_z()
        self._log.info(
            f"[{label}] start_z={start_z * 1000:.1f}mm "
            f"target_z={(start_z + sign * distance_m) * 1000:.1f}mm"
        )

        reached = False
        with self.cartesian_velocity_mode(label):
            self._send_cartesian_velocity(sign * speed_mm_s, label)

            timeout = (distance_m * 1000.0 / speed_mm_s) * DESCENT_TIMEOUT_FACTOR
            loop_start = time.time()

            while True:
                time.sleep(0.02)  # 50 Hz
                elapsed = time.time() - loop_start

                if elapsed > timeout:
                    self._log.warn(
                        f"[{label}] timeout after {elapsed:.1f}s (limit={timeout:.1f}s)"
                    )
                    break

                if self._estop:
                    self._log.warn(f"[{label}] e-stop during descent")
                    raise PickAborted("e-stop during fixed-distance descent")

                current_z = self.tcp_z()
                if current_z is None:
                    continue

                moved = (start_z - current_z) if descend else (current_z - start_z)
                if moved >= distance_m:
                    self._log.info(
                        f"[{label}] reached: moved={moved * 1000:.1f}mm "
                        f"(target={distance_m * 1000:.0f}mm)"
                    )
                    reached = True
                    break

        return reached

    def _set_xarm_mode(self, mode: int) -> bool:
        try:
            if not self._set_mode_client.wait_for_service(timeout_sec=2.0):
                self._log.error("[xArm] set_mode service unavailable")
                return False

            request = SetInt16.Request()
            request.data = mode
            future = self._set_mode_client.call_async(request)
            if not self.wait_for_future(future, timeout=5) or future.result() is None:
                self._log.error(f"[xArm] set_mode({mode}) failed")
                return False

            time.sleep(0.3)

            if not self._set_state_client.wait_for_service(timeout_sec=2.0):
                self._log.error("[xArm] set_state service unavailable")
                return False

            request = SetInt16.Request()
            request.data = 0
            future = self._set_state_client.call_async(request)
            if not self.wait_for_future(future, timeout=5) or future.result() is None:
                self._log.error("[xArm] set_state(0) failed")
                return False

            self._log.info(f"[xArm] mode {mode}, state 0 OK")
            return True

        except Exception as exc:
            self._log.error(f"[xArm] _set_xarm_mode({mode}) error: {exc}")
            return False

    def _stop_cartesian_velocity(self):
        try:
            request = MoveVelocity.Request()
            request.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            request.is_tool_coord = False
            request.duration = 0.0
            if self._cartesian_velocity_client.wait_for_service(timeout_sec=1.0):
                self.wait_for_future(
                    self._cartesian_velocity_client.call_async(request), timeout=5
                )
            self._log.info("[xArm] velocity zeroed")
        except Exception as exc:
            self._log.error(f"[xArm] stop error: {exc}")

    def _restore_mode1(self):
        self._log.info("[xArm] restoring mode 1...")

        for attempt in range(MODE1_RETRY_ATTEMPTS):
            if not self._set_xarm_mode(0):
                self._log.warn(f"[xArm] mode 0 failed (attempt {attempt + 1})")
                time.sleep(1.0)
                continue

            time.sleep(0.5)

            if not self._set_xarm_mode(1):
                self._log.warn(f"[xArm] mode 1 failed (attempt {attempt + 1})")
                time.sleep(1.0)
                continue

            self._log.info(
                f"[xArm] waiting {MODE1_RECOVERY_TIME}s for the trajectory controller"
            )
            time.sleep(MODE1_RECOVERY_TIME)
            self._log.info(f"[xArm] mode 1 restored (attempt {attempt + 1})")
            return

        self._log.error("[xArm] CRITICAL: could not restore mode 1!")
