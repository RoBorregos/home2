#!/usr/bin/env python3
"""Referee: turns Gazebo ground truth, robot speech and task topics into scored events."""

import json
import math
import os
import re

import numpy as np
import rclpy
from frida_constants.hri_constants import (
    DISPLAY_CAPTURE_TOPIC,
    START_BUTTON_CLIENT,
    TASK_STEP_TOPIC,
)
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from frida_task_sim import scenario as scenarios, venue as venues
from frida_task_sim.gz import PoseStream, remove_model
from frida_task_sim.world import WORLD_NAME

ROBOT_MODEL = "frida"
EVENTS_TOPIC = "/sim/referee/events"
FINISH_SERVICE = "/sim/referee/finish"
REST_SPEED = 0.02
REST_TIME = 0.5
LIFT = 0.03
SETTLE_TIMEOUT = 5.0
SEE_RE = re.compile(r"I see an? ([\w ]+?)\.?$", re.IGNORECASE)
SHELF_RE = re.compile(r"The ([\w ]+?) goes on shelf (\d+)", re.IGNORECASE)


class Tracked:
    """Motion state of one object."""

    def __init__(self, obj: dict):
        self.obj = obj
        self.position = np.array(obj["pose"][:3])
        self.stamp = None
        self.rest_since = None
        self.location = obj["surface"]
        self.level = obj["level"]
        self.picked = False
        self.attach_z = 0.0
        self.release = None


class Referee(Node):
    def __init__(self):
        super().__init__("referee")
        self.scenario = scenarios.load(
            self.declare_parameter("scenario_file", "").value
        )
        self.run_dir = self.declare_parameter("run_dir", "").value
        self.venue = venues.load(self.scenario["venue"])
        self.objects = {o["id"]: Tracked(o) for o in self.scenario["objects"]}
        self.poses = PoseStream(WORLD_NAME, names=[*self.objects, ROBOT_MODEL])
        self.t0 = None
        self.held = ""
        self.first_pick = True
        self.at_table = False
        self.recognized: set[str] = set()
        self.events_pub = self.create_publisher(String, EVENTS_TOPIC, 50)
        self.create_subscription(String, "/speech/text_spoken", self._on_speech, 50)
        self.create_subscription(
            String, "/sim/grasp_attached_object", self._on_grasp, 10
        )
        self.create_subscription(Empty, START_BUTTON_CLIENT, self._on_start, 10)
        self.create_subscription(String, TASK_STEP_TOPIC, self._on_step, 10)
        self.create_subscription(String, DISPLAY_CAPTURE_TOPIC, self._on_capture, 10)
        self.create_service(Trigger, FINISH_SERVICE, self._finish)
        self.create_timer(0.1, self._track)
        self.get_logger().info(f"Referee ready ({len(self.objects)} objects)")

    # ── Events ──

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def emit(self, kind: str, **data):
        t = 0.0 if self.t0 is None else self.now() - self.t0
        event = {"t": round(t, 2), "type": kind, **data}
        self.events_pub.publish(String(data=json.dumps(event)))
        if self.run_dir:
            with open(os.path.join(self.run_dir, "events.jsonl"), "a") as f:
                f.write(json.dumps(event) + "\n")

    def _on_start(self, _):
        if self.t0 is None:
            self.t0 = self.now()
            self.emit("start")
            self.door_timer = self.create_timer(
                self.venue["door"]["opens_after_s"], self._open_door
            )

    def _open_door(self):
        self.door_timer.cancel()
        opened = remove_model(WORLD_NAME, venues.DOOR_MODEL)
        self.emit("door_open", opened=opened)

    def _on_step(self, msg: String):
        self.emit("step", name=msg.data)

    def _on_capture(self, msg: String):
        try:
            self.emit("capture", label=json.loads(msg.data).get("label", ""))
        except json.JSONDecodeError:
            pass

    def _on_speech(self, msg: String):
        data = json.loads(msg.data)
        self.emit("speech", who=data["who"], text=data["text"])
        if data["who"] != "robot":
            return
        seen = SEE_RE.search(data["text"].strip())
        if seen:
            self._recognize(seen.group(1).strip().lower().replace(" ", "_"))
        shelf = SHELF_RE.search(data["text"])
        if shelf:
            self._shelf_indicated(
                shelf.group(1).strip().lower().replace(" ", "_"),
                int(shelf.group(2)) - 1,
            )

    # ── Ground truth checks ──

    def robot_xy(self):
        pose = self.poses.get(ROBOT_MODEL)
        return None if pose is None else pose[0][:2]

    def facing(self) -> str:
        """Furniture the robot stands at (nearest within 2 m)."""
        xy = self.robot_xy()
        if xy is None:
            return ""
        best, best_dist = "", 2.0
        for name, f in self.venue["furniture"].items():
            dist = math.hypot(xy[0] - f["center"][0], xy[1] - f["center"][1])
            if dist < best_dist:
                best, best_dist = name, dist
        return best

    def _recognize(self, name: str):
        place = self.facing()
        match = next(
            (
                t
                for oid, t in self.objects.items()
                if t.obj["name"] == name
                and t.location == place
                and oid not in self.recognized
            ),
            None,
        )
        if match:
            self.recognized.add(match.obj["id"])
        self.emit(
            "recognize",
            name=name,
            at=place,
            correct=match is not None,
            object=match.obj["id"] if match else "",
        )

    def _shelf_indicated(self, name: str, level: int):
        category = next(
            (o["category"] for o in self.scenario["objects"] if o["name"] == name), ""
        )
        shelves = self.scenario.get("shelves", {})
        wanted = [int(lv) for lv, cat in shelves.items() if cat == category]
        # A category with no shelf of its own belongs on any free part
        correct = level in wanted if wanted else str(level) not in shelves
        self.emit("shelf_indicated", object=name, level=level, correct=correct)

    def _on_grasp(self, msg: String):
        if msg.data and msg.data in self.objects:
            self.held = msg.data
            tracked = self.objects[msg.data]
            tracked.picked = False
            tracked.attach_z = tracked.position[2]
            tracked.release = None
        elif not msg.data and self.held:
            tracked = self.objects[self.held]
            tracked.release = {
                "t": self.now(),
                "z": tracked.position[2],
                "picked": tracked.picked,
            }
            tracked.rest_since = None
            self.held = ""

    def _track(self):
        now = self.now()
        xy = self.robot_xy()
        table = self.venue["furniture"][self.venue["ppc"]["table"]]
        if xy is not None and not self.at_table and self.t0 is not None:
            if venues.inside(table, xy[0], xy[1], margin=1.0):
                self.at_table = True
                self.emit("at_table")
        for oid, tracked in self.objects.items():
            pose = self.poses.get(oid)
            if pose is None:
                continue
            position = pose[0]
            if tracked.stamp is not None and now > tracked.stamp:
                speed = np.linalg.norm(position - tracked.position) / (
                    now - tracked.stamp
                )
                if speed < REST_SPEED:
                    tracked.rest_since = tracked.rest_since or now
                else:
                    tracked.rest_since = None
            tracked.position, tracked.stamp = position, now
            if oid == self.held:
                if not tracked.picked and position[2] > tracked.attach_z + LIFT:
                    tracked.picked = True
                    self.emit(
                        "pick",
                        object=oid,
                        name=tracked.obj["name"],
                        role=tracked.obj["role"],
                        source=tracked.location,
                        first=self.first_pick,
                    )
                    self.first_pick = False
                continue
            at_rest = (
                tracked.rest_since is not None and now - tracked.rest_since >= REST_TIME
            )
            if tracked.release is not None:
                if at_rest or now - tracked.release["t"] > SETTLE_TIMEOUT:
                    self._placed(oid, tracked)
            elif at_rest:
                location, level = venues.location_of(self.venue, *position)
                if location != tracked.location and location != "air":
                    if location == "floor":
                        self.emit(
                            "knocked",
                            object=oid,
                            name=tracked.obj["name"],
                            source=tracked.location,
                        )
                    tracked.location, tracked.level = location, level
                elif location == tracked.location:
                    tracked.level = level

    def _placed(self, oid: str, tracked: Tracked):
        release, tracked.release = tracked.release, None
        location, level = venues.location_of(self.venue, *tracked.position)
        moved = location != tracked.location or level != tracked.level
        tracked.location, tracked.level = location, level
        if not release["picked"] and not moved:
            return
        self.emit(
            "place",
            object=oid,
            name=tracked.obj["name"],
            role=tracked.obj["role"],
            category=tracked.obj["category"],
            location=location,
            level=level,
            drop=round(float(release["z"] - tracked.position[2]), 3),
        )

    def _finish(self, request, response):
        final = {}
        for oid, tracked in self.objects.items():
            location, level = venues.location_of(self.venue, *tracked.position)
            final[oid] = {
                "position": [round(float(v), 3) for v in tracked.position],
                "location": location,
                "level": level,
            }
        if self.run_dir:
            with open(os.path.join(self.run_dir, "final.json"), "w") as f:
                json.dump(final, f, indent=2)
        self.emit("finish")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Referee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
