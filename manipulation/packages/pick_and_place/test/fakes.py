"""A recording stand-in for RobotArm and Perception, so the pipelines can be
tested without a robot or a ROS graph."""

from contextlib import contextmanager
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PointStamped, PoseStamped

from pick_and_place.pipelines.errors import PickAborted
from pick_and_place.robot.arm import AttachResult, ContactResult


class FakeLogger:
    def __init__(self):
        self.info_messages: List[str] = []
        self.warn_messages: List[str] = []
        self.error_messages: List[str] = []

    def info(self, message):
        self.info_messages.append(str(message))

    def warn(self, message):
        self.warn_messages.append(str(message))

    def warning(self, message):
        self.warn_messages.append(str(message))

    def error(self, message, *args, **kwargs):
        self.error_messages.append(str(message))

    def debug(self, message):
        pass


class FakeObject:
    """Stand-in for a perceived collision object."""

    def __init__(self, height: float = 0.1):
        self.height = height


class FakeArm:
    """Records every call a pipeline makes, and lets tests script the replies.

    ``calls`` is the ordered list of method names, which is what most tests
    assert on: a pick strategy is defined by the sequence of things it does to
    the robot.

    Duck-types :class:`RobotArm` rather than inheriting an ABC;
    ``test_fakes.py`` fails if the two drift apart.
    """

    def __init__(
        self,
        tip_offsets: Optional[Dict[str, float]] = None,
        move_to_pose_results: Optional[List[bool]] = None,
        pregrasp_results: Optional[List[bool]] = None,
        descent_results: Optional[List[bool]] = None,
        contact_results: Optional[List[ContactResult]] = None,
        self_collides: Optional[List[bool]] = None,
        attach_result: Optional[AttachResult] = None,
        named_position_results: Optional[List[bool]] = None,
        abort_after: Optional[int] = None,
    ):
        self.calls: List[str] = []
        self.phases: List[str] = []
        self.contexts: List[str] = []
        self.named_positions: List[str] = []
        self.poses: Dict[str, List[PoseStamped]] = {
            "move_to_pose": [],
            "move_to_pregrasp": [],
            "endpoint_self_collides": [],
        }
        self.descents: List[tuple] = []
        self.close_settles: List[float] = []
        self.guards: List[Any] = []
        self.published_place_poses: List[PoseStamped] = []
        self.published_points: List[PointStamped] = []
        self._logger = FakeLogger()

        self.tf_buffer = None
        self._tip_offsets = tip_offsets or {
            "ee_link_offset": -0.09,
            "rim_tip_offset": -0.18,
            "bowl_tip_offset": -0.12,
        }
        self._move_to_pose_results = list(move_to_pose_results or [])
        self._pregrasp_results = list(pregrasp_results or [])
        self._descent_results = list(descent_results or [])
        self._contact_results = list(contact_results or [])
        self._self_collides = list(self_collides or [])
        self._named_position_results = list(named_position_results or [])
        self._attach_result = attach_result or AttachResult(
            attached=True, lowest_object=FakeObject(), highest_object=FakeObject()
        )
        self._abort_after = abort_after
        self._abort_checks = 0

    @staticmethod
    def _next(queue: List[Any], default: Any) -> Any:
        return queue.pop(0) if queue else default

    # --- motion ------------------------------------------------------------

    def move_to_pose(self, pose, **kwargs):
        self.calls.append("move_to_pose")
        self.poses["move_to_pose"].append(pose)
        return self._next(self._move_to_pose_results, True)

    def move_to_pregrasp(self, pose, velocity=None):
        self.calls.append("move_to_pregrasp")
        self.poses["move_to_pregrasp"].append(pose)
        return self._next(self._pregrasp_results, True)

    def move_to_named_position(self, name, velocity=None):
        self.calls.append("move_to_named_position")
        self.named_positions.append(name)
        return self._next(self._named_position_results, True)

    def move_joints(self, joint_positions, velocity=None):
        self.calls.append("move_joints")
        return True

    def get_joints(self, degrees=True):
        self.calls.append("get_joints")
        return {"joints": {f"joint{i}": 0.0 for i in range(1, 7)}}

    def scan_environment(self):
        self.calls.append("scan_environment")

    def fixed_distance_descent(self, distance_m, speed_mm_s, descend=True):
        self.calls.append("fixed_distance_descent")
        self.descents.append((distance_m, speed_mm_s))
        return self._next(self._descent_results, True)

    def force_guarded_descent(self, guard):
        self.calls.append("force_guarded_descent")
        self.guards.append(guard)
        return self._next(
            self._contact_results, ContactResult(contact=True, descended_m=0.12)
        )

    def wait_until_ready(self):
        self.calls.append("wait_until_ready")

    @contextmanager
    def cartesian_velocity_mode(self, label):
        self.calls.append("cartesian_velocity_mode")
        yield

    def endpoint_self_collides(self, pose):
        self.calls.append("endpoint_self_collides")
        self.poses["endpoint_self_collides"].append(pose)
        return self._next(self._self_collides, False)

    # --- gripper and scene --------------------------------------------------

    def open_gripper(self, settle_s=0.5):
        self.calls.append("open_gripper")

    def close_gripper(self, settle_s=1.5):
        self.calls.append("close_gripper")
        self.close_settles.append(settle_s)

    def clear_octomap(self, settle_s=0.3):
        self.calls.append("clear_octomap")

    def get_collision_objects(self):
        self.calls.append("get_collision_objects")
        return []

    def snapshot_scene(self):
        self.calls.append("snapshot_scene")
        return []

    def remove_collision_object(self, object_id):
        self.calls.append("remove_collision_object")
        return True

    def remove_all_collision_objects(self, attached=False):
        self.calls.append("remove_all_collision_objects")
        return True

    def attach_pick_object(self):
        self.calls.append("attach_pick_object")
        return self._attach_result

    def detach_pick_objects(self):
        self.calls.append("detach_pick_objects")
        return True

    def add_shelf_ceiling_guard(self, place_pose, table_height):
        self.calls.append("add_shelf_ceiling_guard")

    def remove_shelf_ceiling_guard(self):
        self.calls.append("remove_shelf_ceiling_guard")

    def object_pick_height(self, obj, grasp_pose):
        self.calls.append("object_pick_height")
        return 0.05

    def object_height(self, lowest, highest):
        self.calls.append("object_height")
        return 0.12

    # --- state and observability --------------------------------------------

    def tcp_z(self):
        return 0.5

    def now(self):
        return None

    @property
    def estop_active(self):
        return False

    @property
    def joint_state(self):
        return None

    def check_abort(self):
        self._abort_checks += 1
        if self._abort_after is not None and self._abort_checks > self._abort_after:
            raise PickAborted("e-stop active")

    def tip_offset(self, param_name):
        return self._tip_offsets[param_name]

    def wait_for_future(self, future, timeout=60):
        return future

    def bind_goal(self, goal_handle, feedback_factory=None):
        self.calls.append("bind_goal")

    def release_goal(self):
        self.calls.append("release_goal")

    def set_context(self, context):
        self.contexts.append(context)

    @contextmanager
    def phase(self, name):
        self.phases.append(name)
        yield

    def publish_place_pose(self, pose):
        self.published_place_poses.append(pose)

    def publish_debug_point(self, point):
        self.published_points.append(point)

    @property
    def logger(self):
        return self._logger


class FakePerception:
    """Scripted perception replies."""

    def __init__(
        self,
        located_point: Optional[PointStamped] = None,
        cluster=None,
        grasps: Optional[List[tuple]] = None,
        flat_response: Optional[Any] = None,
    ):
        self.calls: List[str] = []
        self._logger = FakeLogger()
        self._located_point = located_point
        self._cluster = cluster
        self._grasps = list(grasps or [])
        self._flat_response = flat_response

    @property
    def logger(self):
        return self._logger

    def locate_object(self, object_name):
        self.calls.append("locate_object")
        return self._located_point

    @staticmethod
    def point_in_range(point, min_distance, max_distance):
        return True

    def cluster_at(self, point, add_collision_objects=True):
        self.calls.append("cluster_at")
        return self._cluster

    def detect_grasps(self, cluster, cfg_path):
        self.calls.append("detect_grasps")
        return self._grasps.pop(0) if self._grasps else ([], [])

    def estimate_flat_grasp(self, object_name):
        self.calls.append("estimate_flat_grasp")
        return self._flat_response


class FakeFlatResponse:
    """Stand-in for the flat-grasp estimator's reply."""

    def __init__(self, pose, samples_collected: int = 5):
        self.pose = pose
        self.samples_collected = samples_collected
        self.success = True
        self.message = ""


def make_pose(x=0.5, y=0.0, z=0.3) -> PoseStamped:
    """A pose whose orientation is identity, so the approach axis is world +Z.

    That keeps the offset arithmetic in the tests readable: applying a tip
    offset of -0.09 moves the pose to z - 0.09.
    """
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def make_point(x=0.5, y=0.0, z=0.3, frame="base_link") -> PointStamped:
    point = PointStamped()
    point.header.frame_id = frame
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


def make_cluster(z_low=0.30, z_high=0.40, frame="base_link"):
    """A real PointCloud2, so the pipeline's height maths runs for real."""
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.frame_id = frame
    points = [
        (0.5, 0.0, z_low),
        (0.5, 0.01, z_high),
        (0.51, 0.0, (z_low + z_high) / 2),
    ]
    return point_cloud2.create_cloud_xyz32(header, points)
