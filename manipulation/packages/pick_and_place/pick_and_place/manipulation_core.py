#!/usr/bin/env python3
"""The manipulation node.

One node owns the whole manipulation task: it serves ManipulationAction,
GoToHand and the face / person follow services, and dispatches each task type
to a pipeline. The pipelines (``pipelines/pick.py``, ``place.py``, ``pour.py``,
``follow.py``) hold the logic and talk to the robot through ``robot/arm.py``
and ``robot/perception.py``.

"""

import copy
import threading
import time

import numpy as np
import rclpy
from frida_constants.manipulation_constants import (
    FIXED_DISTANCE_MOVE_SERVICE,
    FOLLOW_FACE_ARM_SERVICE,
    FOLLOW_PERSON_ARM_SERVICE,
    GO_TO_HAND_ACTION_SERVER,
    GRIPPER_REACH,
    MANIPULATION_ACTION_SERVER,
    RIM_DESCENT_SPEED,
)
from frida_constants.vision_constants import CENTROID_TOPIC, FOLLOW_TOPIC
from frida_interfaces.action import GoToHand, ManipulationAction
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.srv import FixedDistanceMove, FollowFace
from frida_motion_planning.utils.tf_utils import transform_point
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
from transforms3d.quaternions import quat2mat

from pick_and_place.pipelines import follow as follow_pipeline
from pick_and_place.pipelines import pick as pick_pipeline
from pick_and_place.pipelines import place as place_pipeline
from pick_and_place.pipelines import pour as pour_pipeline
from pick_and_place.pipelines.errors import PickAborted, PickHardwareError
from pick_and_place.pipelines.profiles import default_profiles_path, load_profiles
from pick_and_place.pipelines.strategies import PickOutcome, build_strategies
from pick_and_place.robot.arm import RobotArm
from pick_and_place.robot.perception import Perception

# Yaw angles swept when reaching towards a person's hand, in order.
HAND_APPROACH_ANGLES = [0, 180, 200, 220, 240, 270]

FACE_LOOP_PERIOD = 0.1

# The merged node absorbs four nodes' worth of callbacks, and a task callback
# blocks for minutes while it runs.
EXECUTOR_THREADS = 16


class ManipulationCore(Node):
    def __init__(self):
        super().__init__("manipulation_core")
        # One reentrant group for everything. A task callback blocks for the
        # whole pick, so anything on the default mutually-exclusive group could
        # not be serviced while a task runs, and the node would deadlock.
        self.callback_group = ReentrantCallbackGroup()
        self._task_lock = threading.Lock()

        # Physical properties of the gripper; the launch file overrides
        # ee_link_offset. Everything else tuned lives in pick_profiles.yaml.
        self.declare_parameter("ee_link_offset", -0.125)
        self.declare_parameter("rim_tip_offset", -0.18)
        self.declare_parameter("bowl_tip_offset", -0.12)
        self.declare_parameter("geometric_tip_offset", -GRIPPER_REACH)
        # Points at an alternative profile file; empty means the installed one.
        self.declare_parameter("pick_profiles_file", "")

        self.arm = RobotArm(self)
        self.perception = Perception(self)

        # Loaded and validated here so a bad profile fails at launch rather than
        # mid-descent with the arm in cartesian-velocity mode.
        profiles = load_profiles(self._profiles_path())
        self.strategies = build_strategies(profiles)

        # The most recent pick or pour. The place pipeline consumes it, so a
        # place is only meaningful after a pick or pour in this process.
        self._last_pick = PickOutcome()

        self._pipelines = {
            ManipulationTask.PICK: self._run_pick,
            ManipulationTask.PICK_CLOSEST: self._run_pick_closest,
            ManipulationTask.PLACE: self._run_place,
            ManipulationTask.POUR: self._run_pour,
        }

        self._manipulation_server = ActionServer(
            self,
            ManipulationAction,
            MANIPULATION_ACTION_SERVER,
            execute_callback=self.manipulation_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
        )
        self._go_to_hand_server = ActionServer(
            self,
            GoToHand,
            GO_TO_HAND_ACTION_SERVER,
            execute_callback=self.go_to_hand_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
        )
        self._fixed_distance_move_srv = self.create_service(
            FixedDistanceMove,
            FIXED_DISTANCE_MOVE_SERVICE,
            self._fixed_distance_move_cb,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
        )
        self._setup_follow()

        self.arm.wait_until_ready()
        self.get_logger().info(
            f"Manipulation core started with strategies {sorted(self.strategies)}"
        )

    # ==================================================================
    # Parameters
    # ==================================================================

    def _profiles_path(self):
        override = self.get_parameter("pick_profiles_file").value
        return override if override else default_profiles_path()

    # ==================================================================
    # ManipulationAction
    # ==================================================================

    def goal_callback(self, goal_request):
        if not self._task_lock.acquire(blocking=False):
            self.get_logger().warn("Goal rejected: a task is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn("Manipulation task cancellation requested")
        return CancelResponse.ACCEPT

    async def manipulation_callback(self, goal_handle):
        try:
            """Dispatch one manipulation task to its pipeline."""
            request = goal_handle.request
            result = ManipulationAction.Result()
            result.success = 0

            if self.arm.estop_active:
                self.get_logger().warn("E-stop active, aborting manipulation task")
                goal_handle.abort()
                return result

            self._preempt_follow()

            pipeline = self._pipelines.get(request.task_type)
            if pipeline is None:
                self.get_logger().error(f"Unknown task type: {request.task_type}")
                goal_handle.abort()
                return result

            if request.scan_environment:
                self.get_logger().info("Keeping octomap for environment-aware planning")
            else:
                self.arm.clear_octomap()

            self.arm.bind_goal(goal_handle, ManipulationAction.Feedback)
            try:
                success = pipeline(request)
                result.success = int(success)
                if success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            except PickAborted as exc:
                self.get_logger().warn(f"Manipulation task aborted: {exc}")
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            except PickHardwareError as exc:
                self.get_logger().error(
                    f"Manipulation task stopped, hardware fault: {exc}"
                )
                goal_handle.abort()
            except Exception as exc:
                self.get_logger().error(f"Manipulation task failed: {exc}")
                goal_handle.abort()
            finally:
                self.arm.release_goal()

            self.get_logger().info(
                f"[DONE] task {request.task_type} -> {result.success}"
            )
            return result
        finally:
            self._task_lock.release()

    # ==================================================================
    # Pipelines
    # ==================================================================

    def _run_pick(self, request) -> bool:
        params = request.pick_params
        pick_request = pick_pipeline.PickRequest(
            object_name=params.object_name,
            object_point=params.object_point,
            min_distance=params.min_distance or 0.0,
            max_distance=params.max_distance or pick_pipeline.PICK_MAX_DISTANCE,
            is_shelf=request.scan_environment,
            in_configuration=params.in_configuration,
        )

        self.arm.remove_all_collision_objects(attached=True)
        try:
            success, outcome = pick_pipeline.execute(
                self.arm, self.perception, pick_request, self.strategies
            )
        except (PickAborted, PickHardwareError):
            self.arm.remove_all_collision_objects(attached=True)
            raise

        if not success:
            self.arm.remove_all_collision_objects(attached=True)
            self._last_pick = PickOutcome()
            return False

        self.arm.remove_all_collision_objects(attached=False)
        self._last_pick = outcome
        return True

    def _run_pick_closest(self, request) -> bool:
        """PICK_CLOSEST has never been implemented."""
        self.get_logger().error("PICK_CLOSEST is not implemented")
        return False

    def _run_place(self, request) -> bool:
        success = place_pipeline.execute(
            self.arm, self.perception, request.place_params, self._last_pick
        )
        if success:
            self.arm.remove_all_collision_objects(attached=True)
        return success

    def _run_pour(self, request) -> bool:
        params = request.pour_params
        pour_request = pour_pipeline.PourRequest(
            object_name=params.object_name,
            container_name=params.bowl_name,
            object_already_grasped=params.object_already_grasped,
        )

        self.arm.remove_all_collision_objects(attached=True)
        # Clear first: if the pour raises, a stale outcome from an earlier task
        # must not survive for the next place to consume as its drop height.
        self._last_pick = PickOutcome()
        success, self._last_pick = pour_pipeline.execute(
            self.arm, self.perception, pour_request, self.strategies
        )
        if success:
            self.arm.remove_all_collision_objects(attached=True)
        return success

    # ==================================================================
    # GoToHand
    # ==================================================================

    async def go_to_hand_callback(self, goal_handle):
        try:
            return self._go_to_hand(goal_handle)
        finally:
            self._task_lock.release()

    def _go_to_hand(self, goal_handle):
        """Reach towards a person's hand to hand an object over."""
        self._preempt_follow()
        self.get_logger().info("Executing go to hand goal...")
        result = GoToHand.Result()
        result.success = False

        point = copy.deepcopy(goal_handle.request.point)
        self.get_logger().info(
            f"Hand point in frame '{point.header.frame_id}': "
            f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})"
        )

        if point.header.frame_id != "base_link":
            success, point = transform_point(point, "base_link", self.arm.tf_buffer)
            if not success:
                self.get_logger().error(
                    f"Failed to transform hand point from "
                    f"'{goal_handle.request.point.header.frame_id}' to 'base_link'. "
                    "Check that TF is available between these frames."
                )
                goal_handle.succeed()
                return result
            self.get_logger().info(
                f"Transformed to base_link: ({point.point.x:.3f}, "
                f"{point.point.y:.3f}, {point.point.z:.3f})"
            )

        qx, qy, qz, qw = quaternion_from_euler(-np.pi / 2, 0, 0)
        # quat2mat expects scalar-first [w, x, y, z]; the approach axis is the
        # gripper's local Z, the same convention the pick strategies use.
        approach = quat2mat([qw, qx, qy, qz])[:, 2]
        centre = np.array(
            [point.point.x, point.point.y, point.point.z]
        ) + approach * self.arm.tip_offset("ee_link_offset")

        offset = goal_handle.request.hand_offset
        try:
            for angle in HAND_APPROACH_ANGLES:
                pose = PoseStamped()
                pose.header.frame_id = point.header.frame_id
                pose.pose.position.x = centre[0] + offset * np.cos(np.radians(angle))
                pose.pose.position.y = centre[1] + offset * np.sin(np.radians(angle))
                pose.pose.position.z = centre[2]
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                if self.arm.move_to_pose(
                    pose, tolerance_position=0.01, tolerance_orientation=0.1
                ):
                    self.get_logger().info(f"Go to hand pose reached at {angle} deg")
                    result.success = True
                    break

            if not result.success:
                self.get_logger().error("Failed to reach go to hand pose")
            goal_handle.succeed()
            return result

        except Exception as exc:
            self.get_logger().error(f"Go to hand failed: {exc}")
            goal_handle.succeed()
            result.success = False
            return result

    # ==================================================================
    # FixedDistanceMove
    # ==================================================================

    def _fixed_distance_move_cb(self, request, response):
        self._preempt_follow()
        # Service callers get a boolean, not an exception: an e-stop or a mode
        # fault here is a failed move, not a crashed service.
        try:
            response.success = self.arm.fixed_distance_descent(
                request.distance, RIM_DESCENT_SPEED, descend=request.descend
            )
        except (PickAborted, PickHardwareError) as exc:
            self.get_logger().warn(f"[FixedDescent] move failed: {exc}")
            response.success = False
        return response

    # ==================================================================
    # Follow
    # ==================================================================

    def _setup_follow(self):
        self._follow_lock = threading.Lock()
        self._face = follow_pipeline.FaceState()
        self._person = follow_pipeline.PersonState()

        self.create_subscription(
            Point,
            FOLLOW_TOPIC,
            self._face_detection_callback,
            2,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Point,
            CENTROID_TOPIC,
            self._centroid_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_vel_cb,
            10,
            callback_group=self.callback_group,
        )
        self.base_yaw_pub = self.create_publisher(Float64, "/follow/base_yaw", 10)

        self._follow_face_srv = self.create_service(
            FollowFace,
            FOLLOW_FACE_ARM_SERVICE,
            self._follow_face_service_callback,
            callback_group=self.callback_group,
        )
        self._follow_person_srv = self.create_service(
            FollowFace,
            FOLLOW_PERSON_ARM_SERVICE,
            self._follow_person_service_cb,
            callback_group=self.callback_group,
        )

        self.create_timer(
            FACE_LOOP_PERIOD, self._face_loop, callback_group=self.callback_group
        )
        self.create_timer(
            follow_pipeline.PERSON_DT,
            self._person_loop,
            callback_group=self.callback_group,
        )

    def _preempt_follow(self):
        with self._follow_lock:
            if self._face.is_following_face_active:
                follow_pipeline.face_off(self.arm, self._face)
            if self._person.active:
                follow_pipeline.person_off(self.arm, self._person)

    def _follow_face_service_callback(
        self, request: FollowFace.Request, response: FollowFace.Response
    ):
        """Handle follow face service requests."""
        with self._follow_lock:
            if request.follow_face:
                follow_pipeline.face_on(self.arm, self._face)
            else:
                follow_pipeline.face_off(self.arm, self._face)

        response.success = True
        return response

    def _follow_person_service_cb(self, request, response):
        with self._follow_lock:
            if request.follow_face:
                follow_pipeline.person_on(self.arm, self._person)
            else:
                follow_pipeline.person_off(self.arm, self._person)
        response.success = True
        return response

    def _face_detection_callback(self, msg: Point):
        """Receive face position from vision."""
        self._face.face_x = msg.x
        self._face.face_y = msg.y
        self._face.last_face_detection_time = time.time()
        self._face.has_new_face_data = True

    def _centroid_cb(self, msg: Point):
        follow_pipeline.person_centroid(self.arm, self._person, msg)

    def _cmd_vel_cb(self, msg: Twist):
        self._person.base_omega_z = msg.angular.z

    def _face_loop(self):
        """Timer callback: send velocity commands to track the face."""
        follow_pipeline.face_tick(self.arm, self._face)

    def _person_loop(self):
        self._publish_base_yaw(follow_pipeline.person_tick(self.arm, self._person))

    def _publish_base_yaw(self, value: float):
        msg = Float64()
        msg.data = float(value)
        self.base_yaw_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(EXECUTOR_THREADS)
    node = ManipulationCore()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
