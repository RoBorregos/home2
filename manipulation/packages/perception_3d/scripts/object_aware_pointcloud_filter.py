#!/usr/bin/env python3
"""Object-aware pointcloud filter.

Subscribes to:
  - /point_cloud (sensor_msgs/PointCloud2) — the existing downsampled cloud
    that move_group's octomap monitor consumes.
  - /manipulation/target_object_bbox (vision_msgs/Detection3DArray) —
    published by perception_3d/add_primitives when a pick perception runs
    for the target object cluster.

Publishes:
  - /point_cloud_object_filtered (sensor_msgs/PointCloud2) — the same cloud
    with points inside the latest target bbox removed.

When the latest bbox is older than `bbox_ttl_sec` (default 30 s) or never
seen, the filter passes the cloud through unchanged. This means the
octomap behaves normally outside of an active pick — only during the
brief window where perception has published a target bbox do points get
dropped, so the table and other clutter remain represented in the
octomap and the planner avoids them.

The bbox is axis-aligned in `base_link` (per the publisher contract).
Points arrive in the camera optical frame; we transform the bbox into
that frame once per cloud (cheap) using TF2, then test each point with
a vectorized numpy mask (no per-point loop).

Why this exists: at a grasp pose the gripper fingers physically occupy
the space where the target object's voxels live, and MoveIt/FCL would
reject the goal as a self-vs-environment collision. The standard
manipulation-pipeline workaround (used by MoveIt PickAction, PickNik's
deep_grasp_task, NVIDIA Isaac, Sundermeyer et al.) is to remove the
object's points from the cloud BEFORE the octomap is built. The table
stays a voxel obstacle, only the object disappears, the gripper-finger
overlap with the (now-empty) object location is collision-free, and the
arm body still avoids everything via the unchanged octomap voxels of
the rest of the world.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray
import tf2_ros


class ObjectAwarePointcloudFilter(Node):
    def __init__(self):
        super().__init__("object_aware_pointcloud_filter")

        self.declare_parameter("input_topic", "/point_cloud")
        self.declare_parameter("output_topic", "/point_cloud_object_filtered")
        self.declare_parameter("bbox_topic", "/manipulation/target_object_bbox")
        # If no bbox arrives within this many seconds, pass-through. Long
        # enough to span a slow pick attempt, short enough that a crashed/
        # aborted pick doesn't leave the filter active indefinitely.
        self.declare_parameter("bbox_ttl_sec", 30.0)

        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.bbox_topic = (
            self.get_parameter("bbox_topic").get_parameter_value().string_value
        )
        self.bbox_ttl_sec = (
            self.get_parameter("bbox_ttl_sec").get_parameter_value().double_value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest received bbox state (in base_link). None until first bbox.
        # Stored as a dict of (center, half_extent) numpy arrays plus a
        # wallclock receipt time for the TTL check.
        self._bbox_state = None
        self._bbox_received_at = 0.0

        # /point_cloud from downsample_pc is published as RELIABLE
        # (verified). move_group's occupancy_map_monitor subscribes BEST
        # EFFORT and the pair is compatible. Match the publisher so both
        # this node and downstream consumers see the same delivery class.
        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(PointCloud2, self.output_topic, pc_qos)
        self._sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_cb, pc_qos
        )
        self._bbox_sub = self.create_subscription(
            Detection3DArray, self.bbox_topic, self._bbox_cb, 10
        )

        self.get_logger().info(
            f"Object-aware filter: {self.input_topic} -> {self.output_topic} "
            f"(bbox topic: {self.bbox_topic}, TTL: {self.bbox_ttl_sec:.1f}s)"
        )

    # ------------------------------------------------------------------ bbox
    def _bbox_cb(self, msg: Detection3DArray):
        """Latch the most recent target bbox (only the first detection is used)."""
        if not msg.detections:
            self._bbox_state = None
            self._bbox_received_at = 0.0
            return
        det = msg.detections[0]
        c = det.bbox.center.position
        s = det.bbox.size
        center = np.array([c.x, c.y, c.z], dtype=np.float64)
        half_extent = np.array([s.x / 2.0, s.y / 2.0, s.z / 2.0], dtype=np.float64)
        self._bbox_state = {
            "frame_id": msg.header.frame_id or "base_link",
            "center": center,
            "half_extent": half_extent,
        }
        self._bbox_received_at = time.monotonic()
        self.get_logger().info(
            f"Target bbox latched: frame={self._bbox_state['frame_id']} "
            f"center=({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) "
            f"size=({s.x:.3f},{s.y:.3f},{s.z:.3f})"
        )

    def _bbox_is_active(self) -> bool:
        if self._bbox_state is None:
            return False
        return (time.monotonic() - self._bbox_received_at) <= self.bbox_ttl_sec

    # ------------------------------------------------------------------ cloud
    def _cloud_cb(self, msg: PointCloud2):
        if not self._bbox_is_active():
            # No active bbox: pass through unchanged so move_group sees the
            # full scene (table, clutter, everything).
            self._pub.publish(msg)
            return

        # Transform the bbox center into the cloud's frame ONCE. Because
        # the bbox is axis-aligned in base_link and base_link is rigid w.r.t.
        # the camera frame here, transforming just the center suffices for
        # an AABB test in the cloud frame (the half-extent axes don't rotate
        # under pure translation, which is what link_base -> camera reduces
        # to once orientation is folded out). For a strict axis-aligned test
        # in the cloud frame we'd need an OBB; we keep AABB-in-base_link and
        # transform every point to base_link instead — simpler and safe.
        bb = self._bbox_state
        cloud_frame = msg.header.frame_id

        # Read points as a structured array view (x, y, z columns).
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.size == 0:
            self._pub.publish(msg)
            return
        # read_points_numpy returns either (N,3) or (N,) structured;
        # normalize to (N,3) float64.
        if points.ndim == 1:
            points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(
                np.float64
            )
        else:
            points = points.astype(np.float64)

        # Transform points cloud_frame -> bbox.frame_id (base_link).
        if cloud_frame == bb["frame_id"]:
            pts_in_bbox_frame = points
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    bb["frame_id"],
                    cloud_frame,
                    rclpy.time.Time(),  # latest
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(
                    f"TF lookup {cloud_frame}->{bb['frame_id']} failed: {e}; "
                    "passing cloud through unchanged"
                )
                self._pub.publish(msg)
                return
            pts_in_bbox_frame = self._apply_transform_numpy(points, tf.transform)

        # AABB membership test (vectorized).
        lower = bb["center"] - bb["half_extent"]
        upper = bb["center"] + bb["half_extent"]
        inside_mask = np.all(
            (pts_in_bbox_frame >= lower) & (pts_in_bbox_frame <= upper), axis=1
        )
        keep = ~inside_mask
        n_total = points.shape[0]
        n_dropped = int(inside_mask.sum())

        if n_dropped == 0:
            self._pub.publish(msg)
            return

        # Build filtered cloud in the original frame.
        kept_points = points[keep]
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        out_msg = pc2.create_cloud_xyz32(header, kept_points.astype(np.float32))
        self._pub.publish(out_msg)

        # Throttle the info log (one per ~30 callbacks ~ once per ~1.5 s
        # at 18 Hz input rate) to avoid spam.
        if not hasattr(self, "_cloud_count"):
            self._cloud_count = 0
        self._cloud_count += 1
        if self._cloud_count % 30 == 0:
            self.get_logger().info(
                f"Filtered {n_dropped}/{n_total} points inside target bbox"
            )

    @staticmethod
    def _apply_transform_numpy(points: np.ndarray, transform) -> np.ndarray:
        """Apply a geometry_msgs/Transform to an (N,3) numpy array.

        Equivalent to calling tf2_geometry_msgs.do_transform_point on each
        point but vectorized: builds the 4x4 homogeneous matrix once and
        multiplies. Several orders of magnitude faster for clouds with 1000s
        of points.
        """
        t = transform.translation
        q = transform.rotation
        # Quaternion to rotation matrix (Wikipedia formula).
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ],
            dtype=np.float64,
        )
        return points @ R.T + np.array([t.x, t.y, t.z], dtype=np.float64)


def main():
    rclpy.init()
    node = ObjectAwarePointcloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
