"""Ground truth straight from Gazebo: latest world pose of every named entity."""

import subprocess
import threading

import numpy as np
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from scipy.spatial.transform import Rotation


class PoseStream:
    """Subscribes to /world/<world>/pose/info and keeps name -> (position, rotation)."""

    def __init__(self, world: str, names=None, rate: int = 20):
        self.names = set(names) if names is not None else None
        self._poses: dict[str, tuple[np.ndarray, Rotation]] = {}
        self._lock = threading.Lock()
        self.sim_time = 0.0
        self._node = GzNode()
        options = SubscribeOptions()
        options.msgs_per_sec = rate
        self._node.subscribe(
            Pose_V, f"/world/{world}/pose/info", self._on_poses, options
        )

    def _on_poses(self, msg: Pose_V):
        update = {}
        for p in msg.pose:
            if self.names is not None and p.name not in self.names:
                continue
            q = p.orientation
            update[p.name] = (
                np.array([p.position.x, p.position.y, p.position.z]),
                Rotation.from_quat([q.x, q.y, q.z, q.w]),
            )
        with self._lock:
            self._poses.update(update)
            self.sim_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def get(self, name: str):
        with self._lock:
            return self._poses.get(name)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._poses)


def remove_model(world: str, name: str) -> bool:
    """Delete a model from the running world."""
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'name: "{name}", type: MODEL',
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    return "true" in result.stdout
