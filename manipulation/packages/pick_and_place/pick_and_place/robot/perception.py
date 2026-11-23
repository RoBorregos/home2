"""Perception: what the robot can see, as the pipelines see it.

The counterpart to :class:`RobotArm`. Splitting "see" from "do" keeps either
facade from becoming a god object, and keeps the pipelines free of ROS clients.
"""

import time
from typing import List, Optional, Tuple

from frida_constants.manipulation_constants import (
    DETECT_DOOR_HANDLE_SERVICE,
    DETECT_SHELF_SERVICE,
    GENERATE_GRASPS_SERVICE,
    GRASP_DETECTION_SERVICE,
    HEATMAP_PLACE_SERVICE,
    PICK_PERCEPTION_SERVICE,
    PLACE_PERCEPTION_SERVICE,
)
from frida_constants.vision_constants import (
    DETECTION_HANDLER_TOPIC_SRV,
    GALLERY_DETECTIONS_TOPIC,
)
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.msg import ObjectDetectionArray
from frida_interfaces.srv import (
    DetectDoorHandle,
    DetectionHandler,
    DetectShelf,
    GenerateGrasps,
    GraspDetection,
    HeatmapPlace,
    PickPerceptionService,
    PlacePerceptionService,
)
from geometry_msgs.msg import PointStamped

from pick_and_place.utils.grasp_utils import get_grasps
from pick_and_place.utils.perception_utils import (
    get_object_cluster,
    get_object_point,
    point_in_range,
)

# Defaults, in seconds. Callers override where their flow needs different ones.
GENERATE_GRASPS_TIMEOUT = 5.0
DETECT_TIMEOUT = 2.0
CLUSTER_TIMEOUT = 60.0
DETECT_SHELF_TIMEOUT = 30.0
DETECT_DOOR_HANDLE_TIMEOUT = 15.0  # a few fresh frames of up to 2 s each
SERVICE_WAIT = 5.0  # longest wait for a service to appear


class Perception:
    """Object detection, clustering and grasp generation."""

    def __init__(self, node):
        self._node = node
        self._log = node.get_logger()
        group = node.callback_group

        self.detection_handler_client = node.create_client(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV, callback_group=group
        )
        self.pick_perception_client = node.create_client(
            PickPerceptionService, PICK_PERCEPTION_SERVICE, callback_group=group
        )
        self.place_perception_client = node.create_client(
            PlacePerceptionService, PLACE_PERCEPTION_SERVICE, callback_group=group
        )
        self.grasp_detection_client = node.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE, callback_group=group
        )
        self.heatmap_place_client = node.create_client(
            HeatmapPlace, HEATMAP_PLACE_SERVICE, callback_group=group
        )
        self.generate_grasps_client = node.create_client(
            GenerateGrasps, GENERATE_GRASPS_SERVICE, callback_group=group
        )
        self.detect_shelf_client = node.create_client(
            DetectShelf, DETECT_SHELF_SERVICE, callback_group=group
        )
        self.detect_door_handle_client = node.create_client(
            DetectDoorHandle, DETECT_DOOR_HANDLE_SERVICE, callback_group=group
        )
        # Newest gallery point per label (door_handle), straight from its topic.
        self._gallery_points = {}
        node.create_subscription(
            ObjectDetectionArray,
            GALLERY_DETECTIONS_TOPIC,
            self._on_gallery_detections,
            5,
            callback_group=group,
        )

    def _on_gallery_detections(self, msg) -> None:
        for detection in msg.detections:
            self._gallery_points[detection.label_text] = (
                time.time(),
                detection.point3d,
            )

    def gallery_point(self, label: str, wait: float = 3.0) -> Optional[PointStamped]:
        """The next gallery ``label`` point within ``wait`` s, or None."""
        start = time.time()
        while time.time() - start < wait:
            seen = self._gallery_points.get(label)
            if seen is not None and seen[0] >= start:
                return seen[1]
            time.sleep(0.05)
        return None

    @property
    def logger(self):
        return self._log

    def locate_object(
        self, object_name: str, timeout: float = DETECT_TIMEOUT
    ) -> PointStamped:
        """Find an object by label. An empty frame_id means "not found"."""
        return get_object_point(object_name, self.detection_handler_client, timeout)

    def detections(self, timeout: float = DETECT_TIMEOUT) -> list:
        """Every object the detector sees now (``label_text``, ``point3d``)."""
        request = DetectionHandler.Request(label="all")
        response = self._call(self.detection_handler_client, request, timeout)
        return list(response.detection_array.detections) if response else []

    def detect_shelf(
        self,
        reset: bool = False,
        capture: bool = False,
        detect: bool = False,
        timeout: float = DETECT_SHELF_TIMEOUT,
    ):
        """One DetectShelf call: the response, or None when it failed (logged)."""
        request = DetectShelf.Request(reset=reset, capture=capture, detect=detect)
        response = self._call(self.detect_shelf_client, request, timeout)
        if response is None or not response.success:
            reason = response.message if response is not None else "no answer"
            self._log.warn(f"detect_shelf: {reason}")
            return None
        return response

    def detect_door_handle(
        self,
        frames: int = 0,
        seed: Optional[PointStamped] = None,
        timeout: float = DETECT_DOOR_HANDLE_TIMEOUT,
    ):
        """DetectDoorHandle at ``seed`` (else the whole view); None on failure (logged)."""
        request = DetectDoorHandle.Request(frames=frames)
        if seed is not None:
            request.seed = seed
        response = self._call(self.detect_door_handle_client, request, timeout)
        if response is None or not response.success:
            reason = response.message if response is not None else "no answer"
            self._log.warn(f"detect_door_handle: {reason}")
            return None
        return response

    @staticmethod
    def point_in_range(point, min_distance: float, max_distance: float) -> bool:
        return point_in_range(point, min_distance, max_distance)

    def cluster_at(
        self,
        point: PointStamped,
        add_collision_objects: bool = True,
        timeout: float = CLUSTER_TIMEOUT,
        band: Optional[Tuple[float, float]] = None,
    ):
        """Segment the object at ``point`` into a point cloud, or None.

        ``add_collision_objects=False`` is used when only the geometry is
        wanted: the place flow would otherwise add the table as an obstacle and
        make MoveIt reject every near-table path. ``band``: base_link (min_z, max_z)
        to crop to first, 0 keeps the default.
        """
        min_height, max_height = band if band is not None else (0.0, 0.0)
        return get_object_cluster(
            point,
            self.pick_perception_client,
            add_collision_objects,
            timeout,
            min_height=min_height,
            max_height=max_height,
        )

    def detect_grasps(self, cluster, cfg_path: str) -> Tuple[List, List]:
        """Run GPD over a cluster. Returns (poses, scores), possibly empty."""
        return get_grasps(self.grasp_detection_client, cluster, cfg_path)

    def generate_grasps(
        self, object_name: str, timeout: float = GENERATE_GRASPS_TIMEOUT + 3.0
    ) -> Optional[object]:
        """Ask the grasp generator for ranked grasp candidates.

        Returns the service response, or None when unavailable or unsuccessful;
        the caller logs the reason.
        """
        request = GenerateGrasps.Request()
        request.object_name = object_name
        request.num_samples = 0  # let the generator use its default
        response = self._call(self.generate_grasps_client, request, timeout)
        if response is None or not response.success:
            reason = response.message if response is not None else "no response"
            self._log.error(f"Grasp generation failed for {object_name}: {reason}")
            return None
        return response

    def _call(self, client, request, timeout: float):
        """Wait for the service, call it and wait for the answer: the response or None."""
        if not client.wait_for_service(timeout_sec=min(timeout, SERVICE_WAIT)):
            self._log.error(f"{client.srv_name} unavailable")
            return None
        future = wait_for_future(client.call_async(request), timeout=timeout)
        return future.result() if future else None
