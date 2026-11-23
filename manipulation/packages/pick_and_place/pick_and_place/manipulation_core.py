#!/usr/bin/env python3
"""The manipulation node.

One node owns the whole manipulation task: it serves ManipulationAction,
GoToHand, TakeFromHand and the face / person follow services, and dispatches
each task type to a pipeline. The pipelines (``pipelines/pick.py``, ``place.py``,
``pour.py``, ``shelf.py``, ``follow.py``, ``handover.py``) hold the logic and talk to the robot through
``robot/arm.py`` and ``robot/perception.py``.

"""

import copy
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from frida_constants.manipulation_constants import (
    FIXED_DISTANCE_MOVE_SERVICE,
    FOLLOW_FACE_ARM_SERVICE,
    FOLLOW_PERSON_ARM_SERVICE,
    GO_TO_HAND_ACTION_SERVER,
    GRIPPER_REACH,
    MANIPULATION_ACTION_SERVER,
    PICK_ACCELERATION,
    TAKE_FROM_HAND_ACTION_SERVER,
    SHELF_ACCELERATION,
    SHELF_SCAN_TOLERANCE,
    RIM_DESCENT_SPEED,
)
from frida_constants.vision_constants import CENTROID_TOPIC, FOLLOW_TOPIC
from frida_interfaces.action import GoToHand, ManipulationAction, TakeFromHand
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.srv import FixedDistanceMove, FollowFace
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
from transforms3d.quaternions import quat2mat

from pick_and_place.pipelines import follow as follow_pipeline
from pick_and_place.pipelines import handover as handover_pipeline
from pick_and_place.pipelines import pick as pick_pipeline
from pick_and_place.pipelines import place as place_pipeline
from pick_and_place.pipelines import pour as pour_pipeline
from pick_and_place.pipelines import shelf as shelf_pipeline
from pick_and_place.pipelines.classification import PICK_STRATEGY_SHELF
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


PICK_METHODS = ("auto", "geometric", "gpd")


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
        # Benchmark switch: auto, gpd, or geometric (no GPD fallback).
        self.declare_parameter("pick_method", "auto")
        # Shelf speed-ups to try on the robot one at a time (defaults: as validated).
        self.declare_parameter("shelf_acceleration", SHELF_ACCELERATION)
        self.declare_parameter("shelf_place_straight", False)
        self.add_on_set_parameters_callback(self._validate_parameters)

        self.arm = RobotArm(self)
        self.perception = Perception(self)

        # Loaded and validated here so a bad profile fails at launch rather than
        # mid-descent with the arm in cartesian-velocity mode.
        profiles = load_profiles(self._profiles_path())
        self.strategies = build_strategies(profiles)

        # The most recent pick or pour. The place pipeline consumes it, so a
        # place is only meaningful after a pick or pour in this process.
        self._last_pick = PickOutcome()
        # The last SCAN_SHELF, kept while the base stays put; this node alone puts its
        # boxes in the planning scene.
        self._shelf = None
        self._picked_level = 0

        self._pipelines = {
            ManipulationTask.PICK: self._run_pick,
            ManipulationTask.PICK_CLOSEST: self._run_pick_closest,
            ManipulationTask.PLACE: self._run_place,
            ManipulationTask.POUR: self._run_pour,
            ManipulationTask.SCAN_SHELF: self._run_scan_shelf,
        }

        self._manipulation_server = ActionServer(
            self,
            ManipulationAction,
            MANIPULATION_ACTION_SERVER,
            execute_callback=self.general_manipulation_callback,
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
        self._take_from_hand_server = ActionServer(
            self,
            TakeFromHand,
            TAKE_FROM_HAND_ACTION_SERVER,
            execute_callback=self.take_from_hand_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
        )
        self._fixed_distance_move_srv = self.create_service(
            FixedDistanceMove,
            FIXED_DISTANCE_MOVE_SERVICE,
            self._fixed_distance_move_cb,
            callback_group=self.callback_group,
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
        self._picked_level = 0
        try:
            success = pipeline(request)
            result.success = int(success)
            if request.task_type == ManipulationTask.SCAN_SHELF and self._shelf:
                result.shelf_levels = list(self._shelf.levels)
            result.shelf_level = self._picked_level
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
            self.get_logger().error(f"Manipulation task stopped, hardware fault: {exc}")
            goal_handle.abort()
        except Exception as exc:
            self.get_logger().error(f"Manipulation task failed: {exc}")
            goal_handle.abort()
        finally:
            self.arm.release_goal()

        self.get_logger().info(f"[DONE] task {request.task_type} -> {result.success}")
        return result

    async def general_manipulation_callback(self, goal_handle):
        try:
            return await self.manipulation_callback(goal_handle)
        finally:
            self._task_lock.release()

    # ==================================================================
    # Pipelines
    # ==================================================================

    def _validate_parameters(self, parameters):
        """Reject a pick_method typo or an acceleration outside (0, 1]."""
        for parameter in parameters:
            if parameter.name == "pick_method" and parameter.value not in PICK_METHODS:
                return SetParametersResult(
                    successful=False,
                    reason=f"pick_method must be one of {PICK_METHODS}",
                )
            if parameter.name == "shelf_acceleration" and not 0 < parameter.value <= 1:
                return SetParametersResult(
                    successful=False, reason="shelf_acceleration must be in (0, 1]"
                )
        return SetParametersResult(successful=True)

    def _run_scan_shelf(self, request) -> bool:
        """Survey the shelf in front, keep the scan and put its boxes in the scene."""
        self._shelf = None
        self.arm.remove_shelf_scene()
        with self._shelf_motion(True):
            tuning = self.strategies[PICK_STRATEGY_SHELF].profile.shelf
            shelf = shelf_pipeline.scan(self.arm, self.perception, tuning)
        if shelf is None:
            return False
        self._shelf = shelf
        return self.arm.set_shelf_boxes(shelf.boxes)

    def _current_shelf(self):
        """The last scan while the base has not moved since, else None (and no boxes)."""
        if self._shelf is None:
            self.get_logger().error("No shelf scan: run SCAN_SHELF first")
            return None
        stale = self._shelf.stale(shelf_pipeline.base_pose(self.arm))
        if stale:
            self.get_logger().error(f"Shelf scan dropped: {stale}")
            self._shelf = None
            self.arm.remove_shelf_scene()
        return self._shelf

    def _run_pick(self, request) -> bool:
        params = request.pick_params
        method = self.get_parameter("pick_method").value
        shelf = self._current_shelf() if params.from_shelf else None
        if params.from_shelf and shelf is None:
            self._last_pick = PickOutcome()
            return False
        pick_request = pick_pipeline.PickRequest(
            object_name=params.object_name,
            object_point=params.object_point,
            min_distance=params.min_distance or 0.0,
            max_distance=params.max_distance or pick_pipeline.PICK_MAX_DISTANCE,
            is_shelf=params.from_shelf,
            use_gpd=method == "gpd",
            gpd_fallback=method != "geometric",
            # A shelf pick has faced the object's level: perceive from there.
            in_configuration=params.in_configuration or params.from_shelf,
            shelf=shelf,
        )

        self.arm.remove_all_collision_objects(attached=True)
        # The wipe takes the shelf boxes too: a shelf pick puts them back to face the
        # object's level, and the wipes below remove them again.
        if shelf is not None and not self._face_object_level(shelf, params):
            self.arm.remove_all_collision_objects(attached=True)
            self._last_pick = PickOutcome()
            return False
        try:
            with self._shelf_motion(pick_request.is_shelf):
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

    def _face_object_level(self, shelf, params) -> bool:
        """Put the shelf boxes back and face the level that holds the object."""
        if not self.arm.set_shelf_boxes(shelf.boxes):
            return False
        with self._shelf_motion(True):
            level = shelf_pipeline.find_object(
                self.arm, self.perception, shelf, params.object_name, params.shelf_level
            )
        if not level:
            self.get_logger().error(f"{params.object_name} not seen on any shelf level")
            return False
        self._picked_level = level
        return True

    def _shelf_motion(self, is_shelf: bool):
        """The shelf acceleration for a shelf task, the default otherwise."""
        value = self.get_parameter("shelf_acceleration").value if is_shelf else None
        return self.arm.acceleration_scaling(value or PICK_ACCELERATION)

    def _run_pick_closest(self, request) -> bool:
        """PICK_CLOSEST has never been implemented."""
        self.get_logger().error("PICK_CLOSEST is not implemented")
        return False

    def _run_place(self, request) -> bool:
        params = copy.deepcopy(request.place_params)
        level = params.shelf_level
        is_shelf = params.is_shelf or level > 0
        shelf = self._current_shelf() if is_shelf else None
        if is_shelf and shelf is None:
            return False
        if level:
            if not 1 <= level <= len(shelf.levels):
                self.get_logger().error(f"Shelf level {level} is not in the scan")
                return False
            params.is_shelf = True
            params.table_height = shelf.levels[level - 1].height
            params.table_height_tolerance = SHELF_SCAN_TOLERANCE
            self.arm.set_shelf_boxes(shelf.boxes)
            with self._shelf_motion(True):
                faced = shelf_pipeline.face_level(
                    self.arm, shelf, level, for_place=True
                )
            if not faced:
                self.get_logger().error(f"Could not face shelf level {level}")
                return False
        with self._shelf_motion(is_shelf):
            success = place_pipeline.execute(
                self.arm,
                self.perception,
                params,
                self._last_pick,
                straight_in=is_shelf
                and self.get_parameter("shelf_place_straight").value,
                shelf=shelf,
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
            point = self.arm.to_frame(point, "base_link")
            if point is None:
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
    # TakeFromHand
    # ==================================================================

    async def take_from_hand_callback(self, goal_handle):
        try:
            return self._take_from_hand(goal_handle)
        finally:
            self._task_lock.release()

    def _take_from_hand(self, goal_handle):
        """Take a bag from a person's hand, gripping its handle below the wrist."""
        result = TakeFromHand.Result()
        result.success = False
        result.payload_detected = False

        if self.arm.estop_active:
            self.get_logger().warn("E-stop active, aborting take from hand")
            result.message = "e-stop active"
            goal_handle.abort()
            return result

        self._preempt_follow()
        request = goal_handle.request
        self.arm.bind_goal(goal_handle, TakeFromHand.Feedback)
        try:
            outcome = handover_pipeline.execute(
                self.arm,
                handover_pipeline.TakeFromHandRequest(
                    hand_point=request.hand_point,
                    grasp_below_hand=request.grasp_below_hand,
                    release_timeout=request.release_timeout,
                ),
            )
            result.success = outcome.success
            result.payload_detected = outcome.payload_detected
            result.message = outcome.message
            if outcome.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except PickAborted as exc:
            self.get_logger().warn(f"Take from hand aborted: {exc}")
            result.message = f"aborted: {exc}"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except PickHardwareError as exc:
            self.get_logger().error(f"Take from hand stopped, hardware fault: {exc}")
            result.message = f"hardware fault: {exc}"
            goal_handle.abort()
        except Exception as exc:
            self.get_logger().error(f"Take from hand failed: {exc}")
            result.message = f"failed: {exc}"
            goal_handle.abort()
        finally:
            self.arm.release_goal()

        self.get_logger().info(
            f"[DONE] take from hand -> success={result.success} "
            f"payload={result.payload_detected}"
        )
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
