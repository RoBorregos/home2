#! /usr/bin/env python3
# Abstract class used for all diferent model classes

from abc import ABC, abstractmethod
from dataclasses import dataclass
import imutils
import sys
import copy
import pathlib
from typing import List

sys.path.append(str(pathlib.Path(__file__).parent) + "/../include")
from vision_3D_utils import get2DCentroid, get_depth, deproject_pixel_to_point
from frida_interfaces.msg import ObjectDetection
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
import rclpy.time
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
import tf2_ros


class ObjectDectectorParams:
    def __init__(
        self,
        depth_active: bool = None,
        min_score_thresh: float = None,
        camera_frame: str = None,
        flip_image: bool = None,
        camera_info: CameraInfo = None,
        use_zed_transfrom: bool = None,
    ):
        self.depth_active = depth_active
        self.min_score_thresh = min_score_thresh
        self.camera_frame = camera_frame
        self.flip_image = flip_image
        self.camera_info = camera_info
        self.use_zed_transfrom = use_zed_transfrom

    def __str__(self):
        return (
            "depth_active: "
            + str(self.depth_active)
            + " min_score_thresh: "
            + str(self.min_score_thresh)
            + " camera_frame: "
            + str(self.camera_frame)
            + " flip_image: "
            + str(self.flip_image)
            + " camera_info: "
            + str(self.camera_info)
        )


@dataclass
class BBOX:
    x: float = 0
    y: float = 0
    w: float = 0
    h: float = 0
    x1: float = 0
    x2: float = 0
    y1: float = 0
    y2: float = 0


class Detection:
    def __init__(self, label, class_id, confidence):
        self.bbox_: BBOX = BBOX()
        self.label_: str = label
        self.class_id_: int = class_id
        self.confidence_: float = confidence
        self.point_stamped_: PointStamped

    def __str__(self):
        return (
            "label: "
            + self.label_
            + " class_id: "
            + str(self.class_id_)
            + " confidence: "
            + str(self.confidence_)
            + " bbox: "
            + str(self.bbox_)
            + " point_stamped: "
            + str(self.point_stamped_)
        )


class ObjectDectector(ABC):
    def __init__(self, model_path: str, object_detector_params: ObjectDectectorParams):
        self.object_detector_params_ = object_detector_params
        self.model_path_ = model_path
        self.detections_: List[Detection] = []

    # Abstract method
    @abstractmethod
    def _inference(self, frame):
        pass

    def inference(self, frame, depth_image=[], tfBuffer: tf2_ros = None):
        if self.object_detector_params_.flip_image:
            frame = imutils.rotate(frame, 180)

        visual_frame = copy.deepcopy(frame)
        self.detections_: List[Detection] = []
        self._inference(visual_frame)

        # y1, x1, y2, x2
        if self.object_detector_params_.flip_image:
            for detection in self.detections_:
                (
                    detection.bbox_.y2,
                    detection.bbox_.x2,
                    detection.bbox_.y1,
                    detection.bbox_.x1,
                ) = (
                    1 - detection.bbox_.y1,
                    1 - detection.bbox_.x1,
                    1 - detection.bbox_.y2,
                    1 - detection.bbox_.x2,
                )

        return (self.extract3D(depth_image, tfBuffer), self.detections_, visual_frame)

    def extract3D(self, depth_image, tfBuffer: tf2_ros):
        object_set = {}

        if len(depth_image) != 0:
            # TFs
            while (
                not self.object_detector_params_.use_zed_transfrom
                and not tfBuffer.can_transform(
                    self.object_detector_params_.camera_frame,
                    "base_link",
                    rclpy.time.Time().to_msg(),
                )
            ):
                continue

            pose_array = PoseArray()
            pose_array.header.frame_id = self.object_detector_params_.camera_frame
            pose_array.header.stamp = rclpy.time.Time().to_msg()

        for detection in self.detections_:
            if (
                detection.class_id_ not in object_set.keys()
                or object_set[detection.class_id_].confidence_ < detection.confidence_
            ):
                point_3D = PointStamped(
                    header=Header(frame_id=self.object_detector_params_.camera_frame),
                    point=Point(),
                )

                if self.object_detector_params_.depth_active and len(depth_image) != 0:
                    point_2D = get2DCentroid(
                        [
                            detection.bbox_.y1,
                            detection.bbox_.x1,
                            detection.bbox_.y2,
                            detection.bbox_.x2,
                        ],
                        depth_image,
                    )
                    depth = get_depth(depth_image, point_2D)
                    point_3D_ = deproject_pixel_to_point(
                        self.object_detector_params_.camera_info, point_2D, depth
                    )

                    point_3D.point.x = float(point_3D_[0])
                    point_3D.point.y = float(point_3D_[1])
                    point_3D.point.z = float(point_3D_[2])

                    pose_array.poses.append(Pose(position=point_3D.point))

                detection.point_stamped_ = point_3D

                object_set[detection.class_id_] = detection
        # PUBLISH POINT ARRAY
        # TODO: VISUALIZE MARKER
        return list(object_set.values())

    def getDetections(self):
        return self.detections_

    def getFridaDetections(self, detections: List[Detection]):
        object_detection_array = []
        for detection in detections:
            object_detection = ObjectDetection(
                label=detection.class_id_,
                label_text=detection.label_,
                score=detection.confidence_,
                ymin=float(detection.bbox_.x),
                xmin=float(detection.bbox_.y),
                ymax=float(detection.bbox_.w),
                xmax=float(detection.bbox_.h),
                point3d=detection.point_stamped_,
            )
            object_detection_array.append(object_detection)
        return object_detection_array
