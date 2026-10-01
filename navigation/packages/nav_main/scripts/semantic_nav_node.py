#!/usr/bin/env python3
"""Semantic navigation: patrol routes over the active map's tagged viewpoints.

Navigation's half of the semantic map (issue #1268). Vision owns the object map;
this node owns the *where to stand* layer:

* ``PlanPatrol`` — an ordered route over the tagged sublocation poses, annotated
  with the arm "stare" pose and the dwell time each surface needs. Nav plans, the
  caller drives: it navigates to each pose, asks manipulation for the arm pose and
  waits the dwell so vision can confirm what is there.
* scan bookkeeping — which surfaces the base has actually stood in front of, so
  ``mode: "revisit"`` can order by staleness. Persisted across restarts.
* RViz markers for the route.

The sublocation poses are already robot poses (see ``semantic/surfaces.py``), so
nothing here synthesises standoff poses: the operator tagged them with
``map_area_tagger`` and ``nav_central.go_to_area`` already drives to them.
"""

import json
import math
import os
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from frida_constants.navigation_constants import (
    AREAS_SERVICE,
    GLOBAL_COSTMAP_TOPIC,
    MAP_TOPIC,
    PATROL_MARKERS_TOPIC,
    PATROL_SCAN_RADIUS,
    PATROL_SCAN_YAW_TOLERANCE,
    PATROL_STALENESS_TOPIC,
    PLAN_PATROL_SERVICE,
)
from frida_interfaces.srv import MapAreas, PlanPatrol

from nav_main.semantic.route import order_by_staleness, order_route, route_length
from nav_main.semantic.staleness import ScanLog
from nav_main.semantic.surfaces import load_viewpoints
from nav_main.semantic.viewpoints import Grid, validate_viewpoints

MAP_FRAME = "map"
BASE_FRAME = "base_link"


class SemanticNav(Node):
    def __init__(self):
        super().__init__("semantic_nav")
        self.callback_group = ReentrantCallbackGroup()

        self.map_name = self._param("map_name", os.environ.get("MAP_NAME", "").replace(".db", ""))
        self.areas_retry_period = self._param("areas_retry_period", 10.0)
        self.scan_radius = self._param("scan_radius", PATROL_SCAN_RADIUS)
        self.scan_yaw_tolerance = math.radians(
            self._param("scan_yaw_tolerance_deg", PATROL_SCAN_YAW_TOLERANCE)
        )
        self.scan_check_period = self._param("scan_check_period", 1.0)
        self.snapshot_period = self._param("snapshot_period", 10.0)
        self.marker_period = self._param("marker_period", 2.0)
        self.quick_mode_per_area = self._param("quick_mode_per_area", 1)
        snapshot_dir = self._param("snapshot_dir", "")

        # A tagged pose that now sits inside the inflation makes Nav2 reject the
        # goal and stalls the whole patrol, so every viewpoint is checked against
        # the costmap and relocated around its furniture when it is blocked.
        self.validate_with_costmap = self._param("validate_with_costmap", True)
        self.viewpoint_check_radius = self._param("viewpoint_check_radius", 0.25)
        self.viewpoint_max_cost = int(self._param("viewpoint_max_cost", 50))
        self.viewpoint_max_shift = self._param("viewpoint_max_shift", 0.6)
        self.viewpoint_probe_distance = self._param("viewpoint_probe_distance", 0.8)

        self.areas_data = None
        self.viewpoints = []
        self._fetching = False
        self.scan_log = ScanLog(
            map_name=self.map_name,
            path=self._resolve_snapshot_path(snapshot_dir),
        )

        self.tf_buffer = tf2_ros.Buffer()
        # Held so it is not garbage collected: if it is, /tf stops updating.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.areas_client = self.create_client(
            MapAreas, AREAS_SERVICE, callback_group=self.callback_group
        )
        self.plan_patrol_srv = self.create_service(
            PlanPatrol, PLAN_PATROL_SERVICE, self.plan_patrol_callback,
            callback_group=self.callback_group,
        )

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.staleness_pub = self.create_publisher(String, PATROL_STALENESS_TOPIC, latched)
        self.marker_pub = self.create_publisher(MarkerArray, PATROL_MARKERS_TOPIC, latched)

        # Prefer the global costmap (static map + inflation + live obstacles,
        # latched by Nav2 even while paused); fall back to the SLAM/static map.
        self._costmap = None
        self._static_map = None
        self.create_subscription(
            OccupancyGrid, GLOBAL_COSTMAP_TOPIC,
            lambda msg: setattr(self, "_costmap", msg), latched,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC,
            lambda msg: setattr(self, "_static_map", msg), latched,
            callback_group=self.callback_group,
        )

        self.create_timer(
            self.areas_retry_period, self._areas_timer, callback_group=self.callback_group
        )
        self.create_timer(
            self.scan_check_period, self._scan_timer, callback_group=self.callback_group
        )
        self.create_timer(
            self.snapshot_period, self._snapshot_timer, callback_group=self.callback_group
        )
        self.create_timer(
            self.marker_period, self._marker_timer, callback_group=self.callback_group
        )

        # NOT fetched here: _fetch_areas waits on a future that only the
        # executor can complete, and the executor does not exist yet during
        # __init__. A short one-shot timer does the first attempt instead, so
        # the map is loaded ~1 s in rather than after the first retry period.
        self._first_fetch_timer = self.create_timer(
            1.0, self._first_fetch, callback_group=self.callback_group
        )

        self.get_logger().info(
            f"semantic_nav up (map='{self.map_name or 'unset'}', "
            f"snapshot='{self.scan_log.path or 'memory only'}')"
        )

    # --------------------------------------------------------------------- setup

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _resolve_snapshot_path(self, configured: str) -> str:
        """First writable of: the parameter, /workspace/log, ~/.ros.

        Never a package ``share/`` directory: colcon installs those, and a write
        there is lost on the next build.
        """
        candidates = [configured] if configured else []
        candidates += ["/workspace/log/semantic_nav", os.path.expanduser("~/.ros/semantic_nav")]
        name = f"staleness_{self.map_name or 'default'}.json"
        for directory in candidates:
            try:
                os.makedirs(directory, exist_ok=True)
                probe = os.path.join(directory, ".write_test")
                with open(probe, "w") as handle:
                    handle.write("")
                os.remove(probe)
                return os.path.join(directory, name)
            except OSError:
                continue
        self.get_logger().error("No writable snapshot directory; running memory-only")
        return ""

    # ---------------------------------------------------------------- map areas

    def _first_fetch(self):
        """Exactly one fast attempt; retries fall to the slower areas timer.

        Cancelled before the attempt, not after: a failing fetch blocks its thread
        for up to 7 s (service wait + future poll), so a 1 s timer that kept
        retrying would stack callbacks and starve the executor's 4 threads while
        nav_central is still coming up.
        """
        self._first_fetch_timer.cancel()
        if self.areas_data is None:
            self._fetch_areas()

    def _areas_timer(self):
        if self.areas_data is None:
            self._fetch_areas()

    def _fetch_areas(self) -> bool:
        """Load the areas from nav_central. Never spins this node: the executor's
        other threads complete the future (spinning from inside a callback
        corrupts the wait set).

        Guarded against re-entry: the callback group is reentrant, so two timer
        callbacks can overlap while one is blocked waiting for nav_central, and
        both would load the same areas twice.
        """
        if self._fetching:
            return self.areas_data is not None
        if not self.areas_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{AREAS_SERVICE} not available yet; retrying")
            return False

        self._fetching = True
        try:
            future = self.areas_client.call_async(MapAreas.Request())
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            result = future.result() if future.done() else None
        finally:
            self._fetching = False
        if result is None or not result.areas:
            self.get_logger().warn("Areas service returned no data; retrying")
            return False

        try:
            self.areas_data = json.loads(result.areas)
        except ValueError as exc:
            self.get_logger().error(f"Areas JSON invalid: {exc}")
            return False

        self.viewpoints = load_viewpoints(self.areas_data)
        rooms = sorted({vp.area for vp in self.viewpoints})
        self.get_logger().info(
            f"Loaded {len(self.viewpoints)} tagged viewpoints in {len(rooms)} areas: {rooms}"
        )
        restored, why = self.scan_log.load(self._now())
        if restored:
            self.get_logger().info(f"Restored {restored} scan timestamps")
        elif why not in ("", "no snapshot file"):
            self.get_logger().warn(f"Scan log not restored: {why}")
        return True

    # ------------------------------------------------------------- robot pose

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _robot_pose(self):
        """(x, y, yaw) in the map frame, or None when TF is not available."""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    # ---------------------------------------------------------------- timers

    def _scan_timer(self):
        if not self.viewpoints:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        marked = self.scan_log.mark_if_at(
            self.viewpoints, x, y, yaw, self._now(),
            self.scan_radius, self.scan_yaw_tolerance,
        )
        if marked:
            self.get_logger().info(f"Scanned {marked[0]}", throttle_duration_sec=5.0)
            self._publish_staleness()
            self._staleness_published = True

    def _snapshot_timer(self):
        if not self.scan_log.dirty or not self.scan_log.path:
            return
        try:
            self.scan_log.save(self._now())
        except OSError as exc:
            self.get_logger().warn(f"Could not save scan log: {exc}")

    def _marker_timer(self):
        # Staleness goes out on every tick, with or without a marker subscriber:
        # "what have I never looked at?" must be answerable before the first scan,
        # which is exactly when a task manager asks it.
        if self.viewpoints:
            self._publish_staleness()
        if not self.viewpoints or self.marker_pub.get_subscription_count() == 0:
            return
        now = self._now()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, vp in enumerate(self.viewpoints):
            age = self.scan_log.age(vp.key, now)
            # green = just scanned, red = never scanned
            fresh = 0.0 if age == float("inf") else max(0.0, 1.0 - age / 300.0)
            arrow = Marker()
            arrow.header.frame_id = MAP_FRAME
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "viewpoints"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = vp.x
            arrow.pose.position.y = vp.y
            arrow.pose.position.z = 0.1
            arrow.pose.orientation.z = vp.qz
            arrow.pose.orientation.w = vp.qw
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.08, 0.08
            arrow.color.a = 1.0
            arrow.color.r, arrow.color.g, arrow.color.b = 1.0 - fresh, fresh, 0.0
            markers.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = "labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = vp.x
            label.pose.position.y = vp.y
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.a = 1.0
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = f"{vp.name} ({vp.surface_type})"
            markers.markers.append(label)

        self.marker_pub.publish(markers)

    def _publish_staleness(self):
        now = self._now()
        payload = {
            "map_name": self.map_name,
            "viewpoints": [
                {
                    "area": vp.area,
                    "sublocation": vp.name,
                    "surface_type": vp.surface_type,
                    "age_s": None if self.scan_log.age(vp.key, now) == float("inf")
                    else round(self.scan_log.age(vp.key, now), 1),
                }
                for vp in self.viewpoints
            ],
        }
        self.staleness_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------ grid

    def _grid(self) -> Grid | None:
        """The occupancy grid to validate viewpoints against, or None."""
        if not self.validate_with_costmap:
            return None
        msg = self._costmap or self._static_map
        if msg is None:
            return None
        return Grid(
            data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )

    # --------------------------------------------------------------- service

    def plan_patrol_callback(self, request, response):
        response.success = False
        response.error = ""

        if self.areas_data is None and not self._fetch_areas():
            response.error = "areas not available from nav_central"
            self.get_logger().error(f"Plan_patrol -> {response.error}")
            return response

        areas = [a for a in request.areas if a]
        viewpoints = load_viewpoints(self.areas_data, areas=areas or None)
        if not viewpoints:
            response.error = (
                f"no tagged viewpoints for areas={areas or 'all'} "
                "(areas without a polygon are skipped)"
            )
            self.get_logger().warn(f"Plan_patrol -> {response.error}")
            return response

        pose = self._robot_pose()
        if pose is None:
            # Falling back to the first viewpoint only changes the visit order,
            # never the set of poses, so a missing TF degrades instead of failing.
            start = (viewpoints[0].x, viewpoints[0].y)
            self.get_logger().warn("Plan_patrol -> no robot TF; ordering from first viewpoint")
        else:
            start = (pose[0], pose[1])

        mode = (request.mode or "full").strip().lower()
        if mode == "quick":
            per_area: dict[str, list] = {}
            for vp in viewpoints:
                per_area.setdefault(vp.area, []).append(vp)
            viewpoints = [
                vp
                for group in per_area.values()
                for vp in sorted(group, key=lambda v: v.dwell_s, reverse=True)[
                    : max(1, self.quick_mode_per_area)
                ]
            ]

        grid = self._grid()
        if grid is None and self.validate_with_costmap:
            self.get_logger().warn(
                "Plan_patrol -> no costmap yet; returning the tagged poses unchecked",
                throttle_duration_sec=30.0,
            )
        report = validate_viewpoints(
            viewpoints,
            grid,
            radius=self.viewpoint_check_radius,
            max_cost=self.viewpoint_max_cost,
            max_shift=self.viewpoint_max_shift,
            probe_distance=self.viewpoint_probe_distance,
        )
        if report.relocated or report.dropped:
            self.get_logger().warn(f"Plan_patrol -> costmap check: {report.summary}")
        viewpoints = report.kept
        if not viewpoints:
            response.error = "every viewpoint is blocked in the costmap"
            self.get_logger().error(f"Plan_patrol -> {response.error}")
            return response

        if mode == "revisit":
            ordered, total = order_by_staleness(
                viewpoints, self.scan_log.scans, self._now(), start
            )
        else:
            ordered, total = order_route(viewpoints, start)

        if request.max_viewpoints > 0 and len(ordered) > request.max_viewpoints:
            ordered = ordered[: request.max_viewpoints]
            total = route_length(ordered, start)

        stamp = self.get_clock().now().to_msg()
        for vp in ordered:
            goal = PoseStamped()
            goal.header.frame_id = MAP_FRAME
            goal.header.stamp = stamp
            goal.pose.position.x = vp.x
            goal.pose.position.y = vp.y
            goal.pose.orientation.z = vp.qz
            goal.pose.orientation.w = vp.qw
            response.viewpoints.append(goal)
            response.areas_out.append(vp.area)
            response.sublocations.append(vp.name)
            response.arm_poses.append(vp.arm_pose)
            response.dwell_s.append(float(vp.dwell_s))

        response.total_distance = float(total)
        response.success = True
        self.get_logger().info(
            f"Plan_patrol -> mode='{mode}' {len(ordered)} viewpoints, {total:.1f} m, "
            f"dwell {sum(vp.dwell_s for vp in ordered):.0f} s ({report.summary})"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNav()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.scan_log.dirty and node.scan_log.path:
            try:
                node.scan_log.save(node._now())
            except OSError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
