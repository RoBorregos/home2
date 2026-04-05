"""Grasp database: pre-computed grasps for known objects.

Loads grasp poses from YAML files at startup.  At runtime, translates stored
grasps to the detected object position, filters by workspace/table, and
returns them as PoseStamped arrays ready for PickMotion.
"""

import os
import yaml
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header

# Workspace limits (metres from base_link origin)
MAX_REACH = 0.85
# Approach filter: max angle (radians) from straight-down
MAX_APPROACH_ANGLE = 1.05  # ~60 degrees


class GraspDatabase:
    def __init__(self, db_path: str, logger=None):
        self._logger = logger
        self._objects = {}  # {name: {centroid, grasps: [{pos, orient, score, approach}]}}
        self._load(db_path)

    def _load(self, db_path: str):
        """Load all object grasp files from the database directory."""
        objects_dir = os.path.join(db_path, "objects")
        if not os.path.isdir(objects_dir):
            if self._logger:
                self._logger.warn(f"Grasp database not found at {objects_dir}")
            return

        for name in os.listdir(objects_dir):
            grasps_file = os.path.join(objects_dir, name, "grasps.yaml")
            if not os.path.isfile(grasps_file):
                continue
            try:
                with open(grasps_file, "r") as f:
                    data = yaml.safe_load(f)
                if data and "grasps" in data and len(data["grasps"]) > 0:
                    self._objects[name] = {
                        "centroid": np.array(data.get("centroid", [0, 0, 0]), dtype=np.float64),
                        "grasps": data["grasps"],
                    }
                    if self._logger:
                        self._logger.info(
                            f"Grasp DB: loaded {len(data['grasps'])} grasps for '{name}'"
                        )
            except Exception as e:
                if self._logger:
                    self._logger.warn(f"Grasp DB: failed to load {grasps_file}: {e}")

        if self._logger:
            self._logger.info(
                f"Grasp DB: {len(self._objects)} objects loaded"
            )

    def has_object(self, name: str) -> bool:
        return name in self._objects

    def get_grasps(
        self,
        name: str,
        detected_position: PointStamped,
        table_z: float,
    ) -> Tuple[List[PoseStamped], List[float]]:
        """Look up, transform, filter, and return grasps for an object.

        Args:
            name: Object name (must match directory name in database).
            detected_position: 3D position of the detected object (in base_link).
            table_z: Z height of the table surface.

        Returns:
            (grasp_poses, grasp_scores) sorted by score descending.
            Empty lists if object not found or all grasps filtered out.
        """
        if name not in self._objects:
            return [], []

        obj = self._objects[name]
        det_pos = np.array([
            detected_position.point.x,
            detected_position.point.y,
            detected_position.point.z,
        ])

        if self._logger:
            self._logger.info(
                f"Grasp DB lookup: object={name}, "
                f"detected_pos=({det_pos[0]:.3f},{det_pos[1]:.3f},{det_pos[2]:.3f}), "
                f"frame={detected_position.header.frame_id}, table_z={table_z:.3f}"
            )

        # Offset: where the object was when captured vs where it is now
        # Stored grasps are relative to the stored centroid.
        # At runtime, we translate them to the detected position.
        # The centroid height offset is preserved (grasp Z positions are
        # relative, so a grasp at +0.04 from centroid stays at +0.04).
        translation = det_pos - obj["centroid"]
        # Only translate XY — keep the Z offset from the stored centroid
        # because the grasp heights are relative to the table, not the centroid.
        # Actually, translate all 3 axes: the grasp position is stored relative
        # to the centroid, so adding the detected centroid gives the absolute pos.

        poses = []
        scores = []
        n_table = 0
        n_reach = 0
        n_approach = 0

        frame_id = detected_position.header.frame_id or "base_link"

        for g in obj["grasps"]:
            # Absolute position = relative position + detected centroid
            # (stored grasps are relative to the stored centroid, so we
            #  add the DETECTED position directly, not a delta)
            gpos = np.array(g["position"], dtype=np.float64)
            abs_pos = gpos + det_pos

            # Filter: above table
            if abs_pos[2] < table_z + 0.005:
                n_table += 1
                continue

            # Filter: within workspace
            dist = np.sqrt(abs_pos[0] ** 2 + abs_pos[1] ** 2)
            if dist > MAX_REACH:
                n_reach += 1
                continue

            # No approach filter for database grasps — they were already
            # validated by GPD offline.  Let MoveIt decide reachability.

            # Build PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(abs_pos[0])
            pose.pose.position.y = float(abs_pos[1])
            pose.pose.position.z = float(abs_pos[2])

            orient = g["orientation"]  # [x, y, z, w]
            pose.pose.orientation.x = float(orient[0])
            pose.pose.orientation.y = float(orient[1])
            pose.pose.orientation.z = float(orient[2])
            pose.pose.orientation.w = float(orient[3])

            poses.append(pose)
            scores.append(float(g.get("score", 0.5)))

        if self._logger:
            self._logger.info(
                f"Grasp DB filter: total={len(obj['grasps'])}, "
                f"rejected: table={n_table}, reach={n_reach}, approach={n_approach}, "
                f"passed={len(poses)}"
            )

        # Sort by score descending
        if poses:
            paired = sorted(zip(scores, poses), key=lambda x: -x[0])
            scores, poses = zip(*paired)
            scores = list(scores)
            poses = list(poses)

        return poses, scores
