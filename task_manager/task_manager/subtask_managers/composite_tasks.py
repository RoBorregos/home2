"""Subtasks that combine several areas (vision, manipulation, navigation, hri)."""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers transform type)

from frida_constants.manipulation_constants import GRIPPER_GRASP_STATE_TOPIC
from frida_constants.vision_constants import CAMERA_FRAME
from frida_interfaces.msg import GripperGraspState
from task_manager.utils.colored_logger import CLog
from task_manager.utils.grasp_confirmation import count_by_class, picked_ok
from task_manager.utils.shelf_pick_logic import find_target_on_level, height_matches_level
from task_manager.utils.status import Status

SHELF_SETTLE_S = 3.0


class CompositeTasks:
    """Reusable multi-area subtasks; every call composes mockable area primitives."""

    def __init__(self, node, vision, nav, manipulation, hri, tf_buffer=None):
        self.node = node
        self.vision = vision
        self.nav = nav
        self.manipulation = manipulation
        self.hri = hri
        self.tf_buffer = tf_buffer
        self.shelf_level_cache: dict[str, float] = {}
        self.gripper_has_object = False
        node.create_subscription(GripperGraspState, GRIPPER_GRASP_STATE_TOPIC, self._gripper_cb, 10)
        self._clear_octomap_client = node.create_client(Empty, "/clear_octomap")

    def enable_tf(self):
        """Start the TF listener; call early so the buffer is filled before the first lookup."""
        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
            self._tf_listener = TransformListener(self.tf_buffer, self.node)

    def _gripper_cb(self, msg):
        self.gripper_has_object = msg.object_detected

    def wait(self, seconds: float):
        """Sleep while keeping the node's callbacks alive."""
        end = self.node.get_clock().now() + Duration(seconds=seconds)
        while self.node.get_clock().now() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # ── Navigation ──

    def navigate_and_dock(self, room: str, place: str, dock_offset=None, say=True) -> Status:
        """Tuck the arm, drive to an areas.json location and dock if an offset is given."""
        self.manipulation.move_to_position("nav_pose")
        if say:
            self.hri.say(f"Now I will go to the {(place or room).replace('_', ' ')}.", wait=False)
        status, _ = self.nav.move_to_location(room, place)
        if status != Status.EXECUTION_SUCCESS:
            CLog.nav(self.node, "FAIL", f"Could not reach {room}/{place}.", level="error")
            return status
        if dock_offset is not None:
            self.nav.dock_table(offset=dock_offset)
        return Status.EXECUTION_SUCCESS

    # ── Perception helpers ──

    def detection_point(self, detection):
        """base_link (x, y, z) of a detection, or None on TF failure."""
        self.enable_tf()
        try:
            stamped = PointStamped()
            stamped.header.frame_id = CAMERA_FRAME
            stamped.header.stamp = rclpy.time.Time().to_msg()
            stamped.point.x = float(detection.px)
            stamped.point.y = float(detection.py)
            stamped.point.z = float(detection.pz)
            p = self.tf_buffer.transform(stamped, "base_link", timeout=Duration(seconds=1.0))
            return (p.point.x, p.point.y, p.point.z)
        except TransformException as e:
            CLog.vision(self.node, "TF", f"Transform failed: {e}", level="warn")
        except Exception as e:
            CLog.vision(self.node, "TF", f"Error converting point: {e}", level="error")
        return None

    def detections_at_level(self, detections, level_height: float) -> list:
        """Detections whose base_link height belongs to the shelf at level_height."""
        result = []
        for det in detections or []:
            point = self.detection_point(det)
            if point is not None and height_matches_level(point[2], level_height):
                result.append(det)
        return result

    def detect_with_retries(self, retries: int = 3):
        """detect_objects, retrying on service errors."""
        status, dets = self.vision.detect_objects()
        for _ in range(retries):
            if status == Status.EXECUTION_SUCCESS:
                break
            self.wait(1.0)
            status, dets = self.vision.detect_objects()
        return status, dets or []

    def table_counts(self) -> dict:
        """Detections per class at the current view."""
        _, dets = self.vision.detect_objects()
        return count_by_class([d.classname for d in (dets or [])])

    def announce_objects(self, names):
        """Say each object name; every announced instance scores recognize points."""
        for name in names:
            if name and name != "unknown":
                self.hri.say(f"I see a {name}.")

    # ── Gripper ──

    def ensure_gripper_empty(self, held_name, attempts=2, use_grasp_detector=False) -> Status:
        """Place whatever is held at the current spot so a new pick never drops it."""
        if not held_name:
            return Status.EXECUTION_SUCCESS
        if use_grasp_detector and not self.gripper_has_object:
            CLog.manip(self.node, "PICK", "Grasp bit reports empty gripper.")
            return Status.EXECUTION_SUCCESS
        CLog.manip(self.node, "PICK", f"Still holding {held_name}; placing it here.", level="warn")
        self.hri.say("First I will put down the object I am holding.", wait=False)
        for _ in range(attempts):
            if self.manipulation.place() == Status.EXECUTION_SUCCESS:
                return Status.EXECUTION_SUCCESS
            self.wait(1.0)
        CLog.manip(self.node, "PICK", f"Could not put down {held_name}.", level="error")
        return Status.EXECUTION_ERROR

    def _grasp_holds(self, use_grasp_detector: bool) -> bool:
        if not use_grasp_detector:
            return True
        rclpy.spin_once(self.node, timeout_sec=0.5)
        if not self.gripper_has_object:
            self.hri.say("I did not grasp the object.", wait=False)
        return self.gripper_has_object

    # ── Pick ──

    def pick_confirmed(
        self,
        name: str,
        attempts: int = 2,
        vision_confirm: bool = True,
        use_grasp_detector: bool = False,
        scan_environment: bool = False,
        view_pose: str = "table_stare",
    ) -> Status:
        """Pick from a surface; optionally check the grasp bit and re-detect the target is gone."""
        target = name.lower()
        for attempt in range(1, attempts + 1):
            self.manipulation.move_to_position(view_pose)
            before = self.table_counts() if vision_confirm else {}
            status = self.manipulation.pick_object(name, scan_environment=scan_environment)
            if status == Status.EXECUTION_SUCCESS and not self._grasp_holds(use_grasp_detector):
                status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS and before.get(target, 0) > 0:
                self.manipulation.move_to_position(view_pose)
                if not picked_ok(before, self.table_counts(), target):
                    CLog.manip(self.node, "PICK", f"{name} still on the table.", level="warn")
                    status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS:
                return status
            CLog.manip(self.node, "PICK", f"Pick {name} failed {attempt}/{attempts}.", level="warn")
        return Status.EXECUTION_ERROR

    def view_shelf_level(self, height: float):
        """Aim the camera at a shelf level and let the octomap settle."""
        self.manipulation.get_optimal_position_for_plane(
            height, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.wait(SHELF_SETTLE_S)

    def _find_on_level(self, height: float, name: str):
        """Look at a level, cache what sits there and return counts if name is framed."""
        self.view_shelf_level(height)
        status, dets = self.detect_with_retries()
        if status != Status.EXECUTION_SUCCESS or not dets:
            return None
        candidates = []
        for det in dets:
            point = self.detection_point(det)
            if point is None:
                continue
            candidates.append((det.classname, point[2]))
            if det.classname and height_matches_level(point[2], height):
                self.shelf_level_cache[det.classname.lower()] = height
        if find_target_on_level(candidates, name, height) is None:
            return None
        return count_by_class([n for n, h in candidates if height_matches_level(h, height)])

    def _locate_on_shelf(self, name: str, heights: list[float]):
        cached = self.shelf_level_cache.get(name.lower())
        if cached is not None:
            counts = self._find_on_level(cached, name)
            if counts is not None:
                return cached, counts
            self.shelf_level_cache.pop(name.lower(), None)
        for height in sorted(heights):
            counts = self._find_on_level(height, name)
            if counts is not None:
                return height, counts
        return None, None

    def _clear_octomap(self):
        if self._clear_octomap_client.wait_for_service(timeout_sec=2.0):
            self._clear_octomap_client.call_async(Empty.Request())
            self.wait(SHELF_SETTLE_S)

    def pick_from_shelf(
        self,
        name: str,
        heights: list[float],
        attempts: int = 2,
        vision_confirm: bool = True,
        use_grasp_detector: bool = False,
        clear_octomap: bool = False,
        say_name: str = "",
    ) -> Status:
        """Find the object level by level (cached), then pick it keeping that viewing pose."""
        for attempt in range(1, attempts + 1):
            level, before = self._locate_on_shelf(name, heights)
            if level is None:
                CLog.manip(self.node, "PICK", f"{name} not found on any shelf.", level="error")
                return Status.EXECUTION_ERROR
            if attempt == 1:
                self.announce_objects([name])
                if say_name:
                    self.hri.say(f"I will pick the {say_name}.", wait=False)
            if clear_octomap:
                self._clear_octomap()
            status = self.manipulation.pick_object(
                name, in_configuration=True, scan_environment=True
            )
            if status == Status.EXECUTION_SUCCESS and not self._grasp_holds(use_grasp_detector):
                status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS and vision_confirm and before.get(name.lower()):
                self.view_shelf_level(level)
                _, dets = self.vision.detect_objects()
                after = count_by_class([d.classname for d in self.detections_at_level(dets, level)])
                if not picked_ok(before, after, name):
                    CLog.manip(self.node, "PICK", f"{name} still on the shelf.", level="warn")
                    status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS:
                return status
            CLog.manip(self.node, "PICK", f"Shelf pick {name} failed {attempt}/{attempts}.")
        return Status.EXECUTION_ERROR

    # ── Shelves ──

    def scan_shelves(self, heights: list[float], view_floor: float) -> dict[int, list[str]]:
        """Classes seen per level index (0 = lowest); levels below view_floor are seen from it."""
        shelves = {}
        for idx, height in enumerate(sorted(heights)):
            self.hri.say(f"Scanning shelf number {idx + 1}.", wait=False)
            self.view_shelf_level(max(height, view_floor))
            status, dets = self.detect_with_retries()
            names = [d.classname or "unknown" for d in self.detections_at_level(dets, height)]
            shelves[idx] = names if status == Status.EXECUTION_SUCCESS else []
            CLog.vision(self.node, "SHELF", f"Shelf {idx + 1} ({height:.2f} m): {shelves[idx]}")
        return shelves

    def place_on_shelf_fallback(self, heights: list[float], attempts: int = 2, scan=True):
        """Try each shelf height in order, attempts times each. Returns (Status, height)."""
        for i, height in enumerate(heights):
            if scan and i == 0:
                self.view_shelf_level(height)
            for attempt in range(1, attempts + 1):
                # front_stare gives MoveIt a neutral start; the last scan pose has no valid IK
                self.manipulation.move_to_position("front_stare")
                self.manipulation.get_optimal_position_for_plane(
                    height, tolerance=0.1, table_or_shelf=False, approach_plane=False
                )
                status = self.manipulation.place_on_shelf(plane_height=height, tolerance=0.1)
                if status == Status.EXECUTION_SUCCESS:
                    return Status.EXECUTION_SUCCESS, height
                CLog.manip(self.node, "PLACE", f"Shelf {height:.2f} m failed {attempt}/{attempts}.")
        return Status.EXECUTION_ERROR, None

    # ── Human help ──

    def ask_human(self, request: str, question: str, retries: int = 5) -> Status:
        """Ask a person to do something and wait for a spoken yes."""
        self.hri.say(request, wait=True)
        _, answer = self.hri.confirm(
            question, use_keyword=True, retries=retries, wait_between_retries=10.0
        )
        return Status.EXECUTION_SUCCESS if answer == "yes" else Status.EXECUTION_ERROR
