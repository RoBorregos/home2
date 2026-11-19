"""Subtasks that combine several areas (vision, manipulation, navigation, hri)."""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers transform type)

from frida_constants.manipulation_constants import GRIPPER_GRASP_STATE_TOPIC
from frida_constants.vision_constants import CAMERA_FRAME
from frida_interfaces.msg import GripperGraspState
from task_manager.utils.colored_logger import CLog
from task_manager.utils.grasp_confirmation import count_by_class, picked_ok
from task_manager.utils.shelf_pick_logic import MIN_REACHABLE_SHELF_Z
from task_manager.utils.status import Status


class CompositeTasks:
    """Reusable multi-area subtasks; every call composes mockable area primitives."""

    def __init__(self, node, vision, nav, manipulation, hri, tf_buffer=None):
        self.node = node
        self.vision = vision
        self.nav = nav
        self.manipulation = manipulation
        self.hri = hri
        self.tf_buffer = tf_buffer
        self._rooms: dict[str, str] = {}
        # Manipulation's SCAN_SHELF of the shelf in front: [(height, objects)], lowest first
        self._shelf: list[tuple[float, list[str]]] | None = None
        self.gripper_has_object = False
        # The grasp bit is trusted only once the gripper sensor has published
        self.grasp_sensor = False
        node.create_subscription(GripperGraspState, GRIPPER_GRASP_STATE_TOPIC, self._gripper_cb, 10)

    def enable_tf(self):
        """Start the TF listener; call early so the buffer is filled before the first lookup."""
        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
            self._tf_listener = TransformListener(self.tf_buffer, self.node)

    def _gripper_cb(self, msg):
        self.gripper_has_object = msg.object_detected
        self.grasp_sensor = True

    def wait(self, seconds: float):
        """Sleep while keeping the node's callbacks alive."""
        end = self.node.get_clock().now() + Duration(seconds=seconds)
        while self.node.get_clock().now() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # ── Navigation ──

    def room_of(self, place: str) -> str:
        """Room that holds a map sublocation, from the active map's areas."""
        if not self._rooms:
            _, areas = self.nav.retrieve_areas()
            if isinstance(areas, dict):
                # A sublocation in several rooms (e.g. trashbin) resolves to the first one
                for room, subs in areas.items():
                    for sub in subs:
                        self._rooms.setdefault(sub, room)
        return self._rooms.get(place, "")

    def navigate_and_dock(self, place: str, dock: bool = True, say: bool = True) -> Status:
        """Tuck the arm, drive to a map sublocation and dock to its surface."""
        self.manipulation.move_to_position("nav_pose")
        self._shelf = None  # the scan only holds while the base stays put
        if say:
            self.hri.say(f"Now I will go to the {place.replace('_', ' ')}.", wait=False)
        # TODO(navigation): resolve the room from the sublocation inside move_to_location
        status, _ = self.nav.move_to_location(self.room_of(place), place)
        if status != Status.EXECUTION_SUCCESS:
            CLog.nav(self.node, "FAIL", f"Could not reach {place}.", level="error")
            return status
        if dock:
            # TODO(navigation): table_docker picks a safe stand-off per surface on its own
            self.nav.dock_table()
        return Status.EXECUTION_SUCCESS

    def shelf_scan(self, rescan: bool = False):
        """[(height, objects seen)] of the shelf in front, lowest first; scanned once per stop."""
        if self._shelf is None or rescan:
            status, levels = self.manipulation.scan_shelf()
            self._shelf = levels if status == Status.EXECUTION_SUCCESS and levels else None
            if self._shelf is None:
                CLog.manip(self.node, "SHELF", "Shelf scan failed.", level="error")
        return self._shelf

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

    def detect_on_surface(self, view_pose: str = "table_stare"):
        """Aim the camera at the surface in front and detect objects."""
        self.manipulation.move_to_position(view_pose)
        return self.vision.detect_objects()

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
        if use_grasp_detector and self.grasp_sensor and not self.gripper_has_object:
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
        if not (use_grasp_detector and self.grasp_sensor):
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

    def pick_from_shelf(
        self,
        name: str,
        attempts: int = 2,
        vision_confirm: bool = True,
        use_grasp_detector: bool = False,
        say_name: str = "",
    ) -> Status:
        """Pick from the scanned shelf; manipulation finds the level, faces it and backs out."""
        shelf = self.shelf_scan()
        if shelf is None:
            return Status.EXECUTION_ERROR
        target = name.lower()
        seen = [[o.lower() for o in objs] for _, objs in shelf]
        hint = next((i + 1 for i, objs in enumerate(seen) if target in objs), 0)
        self.announce_objects([name])
        if say_name:
            self.hri.say(f"I will pick the {say_name}.", wait=False)
        for attempt in range(1, attempts + 1):
            status = self.manipulation.pick_object(name, from_shelf=True, shelf_level=hint)
            level = self.manipulation.last_shelf_level
            if status == Status.EXECUTION_SUCCESS and not self._grasp_holds(use_grasp_detector):
                status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS and vision_confirm and level:
                # A held object is closer than the camera's minimum depth, so a rescan skips it
                rescan = self.shelf_scan(rescan=True)
                if (
                    rescan
                    and level <= len(rescan)
                    and target in map(str.lower, rescan[level - 1][1])
                ):
                    CLog.manip(self.node, "PICK", f"{name} still on the shelf.", level="warn")
                    status = Status.EXECUTION_ERROR
            if status == Status.EXECUTION_SUCCESS:
                return status
            CLog.manip(self.node, "PICK", f"Shelf pick {name} failed {attempt}/{attempts}.")
        return Status.EXECUTION_ERROR

    # ── Shelves ──

    def place_on_shelf_near_similar(self, name: str, attempts: int = 2) -> Status:
        """Place on the shelf in front, next to objects of the same category."""
        shelf = self.shelf_scan()
        if shelf is None:
            return Status.EXECUTION_ERROR
        shelves = {i: list(objs) for i, (_, objs) in enumerate(shelf)}
        status, _, to_add, _ = self.hri.categorize_objects([name], shelves)
        idx = None
        if status == Status.EXECUTION_SUCCESS:
            idx = next((int(i) for i, names in to_add.items() if name in names), None)
        # The right level is still announced; unreachable ones give way to the nearest reachable
        order = [i for i, (h, _) in enumerate(shelf) if h >= MIN_REACHABLE_SHELF_Z]
        if idx is not None and idx < len(shelf):
            self.hri.say(f"The {name} goes on shelf {idx + 1}.", wait=False)
            order.sort(key=lambda i: abs(shelf[i][0] - shelf[idx][0]))
        for i in order:
            for attempt in range(1, attempts + 1):
                if self.manipulation.place_on_shelf(level=i + 1) == Status.EXECUTION_SUCCESS:
                    shelf[i][1].append(name)
                    return Status.EXECUTION_SUCCESS
                CLog.manip(self.node, "PLACE", f"Shelf {i + 1} failed {attempt}/{attempts}.")
        return Status.EXECUTION_ERROR
