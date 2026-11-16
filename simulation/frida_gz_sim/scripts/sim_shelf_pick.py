#!/usr/bin/env python3
"""Shelf pick check: the keyboard_input -11 flow, end to end, in Gazebo.

1. Measure the shelf (survey poses + DetectShelf), no per-arena heights.
2. Face the requested level at its MEASURED height.
3. Clear the octomap and pick the object with the shelf pick (scan_environment).
4. Check the shelf boards/walls were in the planning scene during the pick, the
   pick succeeded and the object left the shelf.

    ros2 run frida_gz_sim sim_shelf_pick.py --ros-args -p scene:=competition \
        -p level:=2 -p label:=bottle -p video:=/workspace/sim_logs/shelf_pick.mp4

Needs the world from ``shelf_world.py <scene> <0|1> <out.sdf> pick`` and the sim
vision stack (sim_vision.launch.py) for the detection.
"""

import json
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from frida_constants.manipulation_constants import (
    GET_COLLISION_OBJECTS_SERVICE,
    MANIPULATION_ACTION_SERVER,
    SHELF_COLLISION_NAMESPACE,
)
from frida_constants.vision_constants import CAMERA_ROTATION_TOPIC, DETECTIONS_TOPIC
from frida_gz_sim.shelf_scenes import PICK_MODEL, SCENES, pick_object_pose
from frida_gz_sim.video import VideoRecorder
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask, ObjectDetectionArray
from frida_interfaces.srv import GetCollisionObjects
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task

LIFT_MIN = 0.20  # carried: at least this far above its board at the end (a drop
# back onto the shelf or onto clutter must not count)
PICK_OBJECT_HALF_HEIGHT = 0.095  # the mustard bottle is 19 cm tall


class SceneWatcher(Node):
    """Polls the planning scene during the pick: which shelf boxes were present."""

    def __init__(self):
        super().__init__("shelf_pick_scene_watcher")
        self._client = self.create_client(
            GetCollisionObjects,
            GET_COLLISION_OBJECTS_SERVICE,
            callback_group=ReentrantCallbackGroup(),
        )
        self.watching = False
        self.seen: set = set()
        self.samples = 0
        self.labels_at: dict = {}  # detected label -> wall time last seen
        self.create_timer(0.5, self._poll)
        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self._on_detections,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def _on_detections(self, msg):
        now = time.time()
        for detection in msg.detections:
            self.labels_at[detection.label_text] = now

    def wait_for_label(self, label: str, timeout: float) -> bool:
        """True once ``label`` is detected after this call (CPU YOLO lags motions)."""
        since = time.time()
        while time.time() - since < timeout:
            if self.labels_at.get(label, 0.0) > since:
                return True
            time.sleep(0.1)
        return False

    def _poll(self):
        if not self.watching or not self._client.service_is_ready():
            return
        future = self._client.call_async(GetCollisionObjects.Request())
        future.add_done_callback(self._on_objects)

    def _on_objects(self, future):
        result = future.result()
        if result is None:
            return
        self.samples += 1
        self.seen |= {
            obj.id
            for obj in result.collision_objects
            if obj.id.startswith(SHELF_COLLISION_NAMESPACE)
        }


def object_z(name: str) -> float | None:
    """Ground-truth Z of a gz model, from the gz CLI."""
    try:
        out = subprocess.run(
            ["gz", "model", "-m", name, "-p"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        ).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in out.splitlines():
        parts = line.replace("[", " ").replace("]", " ").split()
        if len(parts) == 3:
            try:
                return float(parts[2])
            except ValueError:
                continue
    return None


def pick_at(node: Node, label: str, point, timeout: float = 180.0) -> Status:
    """Shelf pick at a given base_link point (the -11 flow, detection skipped)."""
    client = ActionClient(node, ManipulationAction, MANIPULATION_ACTION_SERVER)
    if not client.wait_for_server(timeout_sec=10.0):
        return Status.SERVICE_CHECK
    goal = ManipulationAction.Goal()
    goal.task_type = ManipulationTask.PICK
    goal.scan_environment = True
    goal.pick_params.object_name = label
    goal.pick_params.in_configuration = True
    target = PointStamped()
    target.header.frame_id = "base_link"
    target.point.x, target.point.y, target.point.z = map(float, point)
    goal.pick_params.object_point = target
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        return Status.EXECUTION_ERROR
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout)
    if not result.done() or not result.result().result.success:
        return Status.EXECUTION_ERROR
    return Status.EXECUTION_SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = Node("sim_shelf_pick")
    scene = SCENES[node.declare_parameter("scene", "competition").value]
    level = node.declare_parameter("level", 2).value
    objects = node.declare_parameter("objects", True).value
    label = node.declare_parameter("label", "bottle").value
    video = node.declare_parameter("video", "").value
    manipulation = ManipulationTasks(node, task=Task.PICK_AND_PLACE)
    clear_octomap = node.create_client(Empty, "/clear_octomap")
    rotate = node.create_publisher(Int16, CAMERA_ROTATION_TOPIC, 10)

    watcher = SceneWatcher()
    recorder = VideoRecorder(video) if video else None
    executor = MultiThreadedExecutor()
    executor.add_node(watcher)
    if recorder:
        executor.add_node(recorder)
    threading.Thread(target=executor.spin, daemon=True).start()

    def show(status, *lines):
        node.get_logger().info(" | ".join((status, *lines)))
        if recorder:
            recorder.status, recorder.lines = status, list(lines)

    report = {"scene": scene.name, "level": level, "label": label}
    started = time.time()
    z_before = object_z(PICK_MODEL[0])

    # 1. measure the shelf
    show(f"{scene.name}: measuring the shelf")
    status, heights = manipulation.detect_shelf_levels()
    report["heights"] = heights
    report["scan_seconds"] = round(time.time() - started, 1)
    if status != Status.EXECUTION_SUCCESS or len(heights) < level:
        report["error"] = f"shelf detection failed: {status.name} {heights}"
        return finish(node, recorder, executor, video, report, False, show)
    height = heights[level - 1]
    truth = scene.boards[level - 1]
    report["level_height"] = height
    report["level_truth"] = truth

    # 2. face the level at the measured height (what -11 now does)
    show(
        f"{scene.name}: facing level {level} at {height:.3f} m (measured)",
        f"measured levels: {[round(h, 3) for h in heights]}",
    )
    manipulation.get_optimal_position_for_plane(
        height, tolerance=0.1, table_or_shelf=False, approach_plane=True
    )

    # 3. clear the octomap (as -11 does) and pick with the shelf scene restored
    if clear_octomap.wait_for_service(timeout_sec=2.0):
        future = clear_octomap.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    time.sleep(2.0)
    # From the level pose the wrist rolls the ZED image upside down and YOLO misses
    # the object often; the repo's image_orienter fixes that when told the rotation.
    report["label_seen"] = watcher.wait_for_label(label, timeout=10.0)
    report["image_rotation"] = 0
    if not report["label_seen"]:
        rotate.publish(Int16(data=180))
        report["image_rotation"] = 180
        report["label_seen"] = watcher.wait_for_label(label, timeout=20.0)
    point = None
    if not report["label_seen"]:
        # The sim detector is COCO YOLO, not the robot's model: when it misses, hand
        # the pick the object's true position so the grasp itself is still checked.
        x, y, z = pick_object_pose(scene, level - 1, objects)
        point = (x, y, z + PICK_OBJECT_HALF_HEIGHT)
        node.get_logger().warn(
            f"'{label}' not detected from the level pose; picking at its true "
            f"position ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})"
        )
    report["object_point"] = "true position" if point else "detection"
    show(
        f"{scene.name}: picking the {label} on level {level}",
        f"measured levels: {[round(h, 3) for h in heights]}",
        f"object from: {report['object_point']}",
    )
    watcher.watching = True
    pick_started = time.time()
    if point is None:
        pick_status = manipulation.pick_object(
            label, in_configuration=True, scan_environment=True
        )
    else:
        pick_status = pick_at(node, label, point)
    watcher.watching = False
    rotate.publish(Int16(data=0))
    report["pick_seconds"] = round(time.time() - pick_started, 1)
    report["pick_status"] = pick_status.name
    report["shelf_boxes_during_pick"] = sorted(watcher.seen)
    report["scene_samples"] = watcher.samples

    # 4. did the object leave the shelf?
    manipulation.move_to_position("front_stare", velocity=0.5)
    time.sleep(2.0)
    z_after = object_z(PICK_MODEL[0])
    report["object_z"] = [z_before, z_after]
    lifted = (
        z_before is not None and z_after is not None and z_after - z_before > LIFT_MIN
    )
    boards = [b for b in watcher.seen if "board" in b]
    passed = (
        pick_status == Status.EXECUTION_SUCCESS
        and lifted
        and len(boards) >= level  # its board and the one above (the ceiling)
    )
    report["seconds"] = round(time.time() - started, 1)
    return finish(node, recorder, executor, video, report, passed, show)


def finish(node, recorder, executor, video, report, passed, show):
    report["passed"] = passed
    lines = [
        f"measured {[round(h, 3) for h in report.get('heights', [])]}",
        f"shelf boxes in the scene during the pick: {len(report.get('shelf_boxes_during_pick', []))}",
        f"pick {report.get('pick_status', '-')}, object z {report.get('object_z', '-')}",
    ]
    show(f"{report['scene']} shelf pick: {'PASS' if passed else 'FAIL'}", *lines)
    out = Path(video).with_suffix(".json") if video else Path("/tmp/shelf_pick.json")
    out.write_text(json.dumps(report, indent=2))
    node.get_logger().info(f"report: {out} {json.dumps(report)}")
    if recorder:
        time.sleep(3.0)
        recorder.close()
    executor.shutdown()
    rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
