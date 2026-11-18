"""Perception: what the robot can see, as the pipelines see it.

The counterpart to :class:`RobotArm`. Splitting "see" from "do" keeps either
facade from becoming a god object, and keeps the pipelines free of ROS clients.
"""

from typing import List, Optional, Tuple

from frida_constants.manipulation_constants import (
    DETECT_SHELF_SERVICE,
    GENERATE_GRASPS_SERVICE,
    GRASP_DETECTION_SERVICE,
    HEATMAP_PLACE_SERVICE,
    PICK_PERCEPTION_SERVICE,
    PLACE_PERCEPTION_SERVICE,
)
from frida_constants.vision_constants import DETECTION_HANDLER_TOPIC_SRV
from frida_interfaces.srv import (
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
        from frida_motion_planning.utils.ros_utils import wait_for_future

        if not self.detection_handler_client.wait_for_service(timeout_sec=timeout):
            self._log.warn("detection_handler unavailable")
            return []
        request = DetectionHandler.Request(label="all")
        future = wait_for_future(
            self.detection_handler_client.call_async(request), timeout=timeout
        )
        response = future.result() if future else None
        return list(response.detection_array.detections) if response else []

    def detect_shelf(
        self,
        reset: bool = False,
        capture: bool = False,
        detect: bool = False,
        timeout: float = DETECT_SHELF_TIMEOUT,
    ):
        """One DetectShelf call: the response, or None when it failed (logged)."""
        from frida_motion_planning.utils.ros_utils import wait_for_future

        if not self.detect_shelf_client.wait_for_service(timeout_sec=3.0):
            self._log.error("detect_shelf service unavailable")
            return None
        request = DetectShelf.Request(reset=reset, capture=capture, detect=detect)
        future = wait_for_future(
            self.detect_shelf_client.call_async(request), timeout=timeout
        )
        response = future.result() if future else None
        if response is None or not response.success:
            reason = response.message if response is not None else "no answer"
            self._log.warn(f"detect_shelf: {reason}")
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
        from frida_motion_planning.utils.ros_utils import wait_for_future

        if not self.generate_grasps_client.wait_for_service(timeout_sec=5.0):
            self._log.error("generate_grasps service unavailable")
            return None

        request = GenerateGrasps.Request()
        request.object_name = object_name
        request.num_samples = 0  # let the generator use its default
        future = self.generate_grasps_client.call_async(request)
        future = wait_for_future(future, timeout=timeout)
        response = future.result() if future else None

        if response is None or not response.success:
            reason = response.message if response is not None else "no response"
            self._log.error(f"Grasp generation failed for {object_name}: {reason}")
            return None
        return response
