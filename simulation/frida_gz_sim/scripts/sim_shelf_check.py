#!/usr/bin/env python3
"""Check the model-free shelf detection against a Gazebo shelf scene.

Runs the same ``ManipulationTasks.detect_shelf_levels()`` the pick-and-place task
manager calls (survey poses -> capture -> detect), times it and grades the levels
against the scene's ground truth from ``frida_gz_sim.shelf_scenes``.

    ros2 run frida_gz_sim sim_shelf_check.py --ros-args -p scene:=competition \
        -p objects:=true -p video:=/workspace/sim_logs/shelf.mp4

Exit code 0 when every surface is found within ``tolerance`` and nothing extra is
reported. The result also lands as JSON next to the video (or in /tmp).
"""

import json
import threading
import time
from pathlib import Path

import rclpy
from frida_gz_sim.shelf_scenes import SCENES
from frida_gz_sim.video import VideoRecorder
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task


class ShelfCheck(Node):
    def __init__(self):
        super().__init__("sim_shelf_check")
        self.scene = SCENES[self.declare_parameter("scene", "competition").value]
        self.objects = self.declare_parameter("objects", True).value
        self.tolerance = self.declare_parameter("tolerance", 0.015).value
        self.video = self.declare_parameter("video", "").value
        self.manipulation = ManipulationTasks(self, task=Task.PICK_AND_PLACE)

    def grade(self, detected: list) -> dict:
        """Match each true surface to the nearest detection within tolerance."""
        truth = self.scene.surfaces
        remaining = list(detected)
        matches, missing = [], []
        for z in truth:
            best = min(remaining, key=lambda h: abs(h - z), default=None)
            if best is not None and abs(best - z) <= self.tolerance:
                matches.append({"truth": z, "detected": best, "error": best - z})
                remaining.remove(best)
            else:
                missing.append(z)
        return {"matches": matches, "missing": missing, "extra": remaining}


def main(args=None):
    rclpy.init(args=args)
    check = ShelfCheck()
    recorder = VideoRecorder(check.video) if check.video else None
    if recorder is not None:
        executor = MultiThreadedExecutor()
        executor.add_node(recorder)
        threading.Thread(target=executor.spin, daemon=True).start()

    label = f"{check.scene.name} / {'with' if check.objects else 'no'} objects"
    if recorder:
        recorder.status = f"{label}: surveying the shelf"
        recorder.lines = [f"truth: {[round(z, 3) for z in check.scene.surfaces]}"]
        time.sleep(2.0)

    started = time.time()
    status, heights = check.manipulation.detect_shelf_levels()
    elapsed = time.time() - started
    geometry = check.manipulation._call_detect_shelf()  # last result, no capture

    report = {
        "scene": check.scene.name,
        "objects": check.objects,
        "status": status.name,
        "seconds": round(elapsed, 2),
        "truth": check.scene.surfaces,
        "detected": heights,
        **check.grade(heights),
    }
    if geometry is not None and geometry.success:
        report["extent"] = {
            "y": [geometry.y_min, geometry.y_max],
            "x": [geometry.front_x, geometry.back_x],
            "top_z": geometry.top_z,
            "boxes": [obj.id for obj in geometry.collision_objects],
        }
    passed = (
        status == Status.EXECUTION_SUCCESS
        and not report["missing"]
        and not report["extra"]
    )
    report["passed"] = passed

    summary = (
        f"{label}: {'PASS' if passed else 'FAIL'} in {elapsed:.1f} s | detected "
        f"{[round(h, 3) for h in heights]} | missing {report['missing']} | extra "
        f"{[round(h, 3) for h in report['extra']]}"
    )
    check.get_logger().info(summary)
    out = (
        Path(check.video).with_suffix(".json")
        if check.video
        else Path(f"/tmp/shelf_check_{check.scene.name}_{int(check.objects)}.json")
    )
    out.write_text(json.dumps(report, indent=2))
    check.get_logger().info(f"report: {out}")
    if recorder:
        recorder.status = (
            f"{label}: {'PASS' if passed else 'FAIL'} ({elapsed:.1f} s scan)"
        )
        recorder.lines = [
            f"truth:    {[round(z, 3) for z in check.scene.surfaces]}",
            f"detected: {[round(h, 3) for h in heights]}",
            "errors (mm): "
            + ", ".join(f"{m['error'] * 1000:+.0f}" for m in report["matches"]),
        ]
        check.manipulation.move_to_position("front_stare", velocity=0.5)
        time.sleep(3.0)
        recorder.close()
        executor.shutdown()
    rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
