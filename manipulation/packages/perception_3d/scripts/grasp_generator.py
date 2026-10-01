#!/usr/bin/env python3

import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.manipulation_constants import (
    GENERATE_GRASPS_SERVICE,
    GRASP_CLASS_GENERIC,
    GRASP_CLASS_PEAK,
    GRASP_CLASS_RIM,
    GRASP_LINK_FRAME,
    PICK_OBJECT_NAMESPACE,
)
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DETECTIONS_TOPIC,
)
from frida_interfaces.msg import CollisionObject, ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import GenerateGrasps
from perception_3d.geometric_grasps import (
    DEFORMABLE_OBJECTS,
    OBJECT_GRASP_CLASS,
    Fit,
    Grasp,
    GraspRejected,
    Intrinsics,
    Scene,
    classify,
    describe,
    grasp_for,
    parse_bbox,
)

BASE_FRAME = "link_base"  # every grasp pose is expressed here (z up)
DEBUG_POSE_TOPIC = "/manipulation/generated_grasp_pose"
DEFAULT_SAMPLES = 10  # frames to collect when the caller passes <= 0
COLLECT_TIMEOUT = 5.0  # s, give up collecting after this


# One frame's result. measured_class is what geometry alone said (for logs).
@dataclass(frozen=True)
class Sample:
    grasp_class: str
    measured_class: Optional[str]
    fit: Optional[Fit]
    grasps: list


class GraspGenerator(Node):
    def __init__(self):
        super().__init__("grasp_generator")
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._depth = None
        self._depth_frame = CAMERA_FRAME
        self._intrinsics: Optional[Intrinsics] = None

        # Session state: only filled while a service call is collecting.
        self._collecting = False
        self._target = ""
        self._known: Optional[str] = None
        self._samples: list[Sample] = []
        self._rejections: Counter = Counter()
        self._busy = threading.Lock()

        # Reentrant + MultiThreadedExecutor: the service waits while the
        # depth/detection callbacks keep filling samples in other threads.
        group = ReentrantCallbackGroup()
        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._depth_cb, 10, callback_group=group
        )
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 1)
        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self._detections_cb,
            10,
            callback_group=group,
        )
        self._debug_pub = self.create_publisher(PoseStamped, DEBUG_POSE_TOPIC, 10)
        self.create_service(
            GenerateGrasps,
            GENERATE_GRASPS_SERVICE,
            self._generate_cb,
            callback_group=group,
        )
        self.get_logger().info("Grasp generator ready")

    # collect up to N samples for object_name, then combine them.
    def _generate(self, request, response):
        if self._intrinsics is None:
            return self._fail(response, "camera intrinsics not received yet")

        self._start(request.object_name.lower())
        wanted = request.num_samples if request.num_samples > 0 else DEFAULT_SAMPLES
        deadline = time.time() + COLLECT_TIMEOUT
        while time.time() < deadline and len(self._samples) < wanted:
            time.sleep(0.05)
        self._collecting = False

        # Nothing worked: report the most frequent rejection reason.
        if not self._samples:
            reason = self._rejections.most_common(1)
            detail = f"{reason[0][0]} (x{reason[0][1]})" if reason else "not detected"
            return self._fail(response, f"no grasp for '{self._target}': {detail}")
        return self._finish(response)

    def _generate_cb(self, request, response):
        if not self._busy.acquire(blocking=False):
            return self._fail(response, "busy: another request is collecting")
        try:
            return self._generate(request, response)
        finally:
            self._busy.release()

    # Reset the session; known is None for objects not in the lookup table.
    def _start(self, target: str) -> None:
        self._target = target
        self._known = OBJECT_GRASP_CLASS.get(target)
        self._samples = []
        self._rejections = Counter()
        self._depth = None  # never reuse a frame from a previous request
        self._collecting = True

    # Majority class wins. Per approach: median position over the frames that
    # produced it (robust to bad frames), orientation and fit from the latest,
    def _finish(self, response):
        samples = list(self._samples)
        grasp_class = Counter(s.grasp_class for s in samples).most_common(1)[0][0]
        voted = [s for s in samples if s.grasp_class == grasp_class]
        measured = Counter(s.measured_class for s in samples)
        by_approach = defaultdict(list)
        for sample in voted:
            for grasp in sample.grasps:
                by_approach[grasp.approach].append(grasp)
        ranked = sorted(
            (
                Grasp(
                    np.median([g.position for g in grasps], axis=0),
                    grasps[-1].orientation,
                    approach,
                    sum(g.score for g in grasps) / len(voted),
                )
                for approach, grasps in by_approach.items()
            ),
            key=lambda grasp: grasp.score,
            reverse=True,
        )

        response.grasps = [self._pose(g.position, g.orientation) for g in ranked]
        response.scores = [float(g.score) for g in ranked]
        response.approaches = [g.approach for g in ranked]
        response.pose = response.grasps[0]
        if voted[-1].fit is not None:
            response.object = self._collision_object(voted[-1].fit)
        response.grasp_class = grasp_class
        response.deformable = self._target in DEFORMABLE_OBJECTS
        response.samples_collected = len(voted)
        response.success = True
        response.message = (
            f"'{self._target}' as {grasp_class} ({ranked[0].approach}, score "
            f"{ranked[0].score:.2f}, {len(ranked)} candidates) from {len(voted)}/"
            f"{len(samples)} samples, known={self._known}, measured={dict(measured)}"
        )
        self.get_logger().info(response.message)
        return response

    def _collision_object(self, fit: Fit) -> CollisionObject:
        shape = CollisionObject()
        shape.id = PICK_OBJECT_NAMESPACE + self._target
        shape.type = fit.kind
        shape.pose = self._pose(fit.center, Rotation.from_euler("z", fit.yaw).as_quat())
        size = shape.dimensions
        size.x, size.y, size.z = map(float, fit.size)
        return shape

    def _fail(self, response, message: str):
        self.get_logger().warn(message)
        response.success = False
        response.message = message
        return response

    # Idle unless a request is collecting (saves CPU).
    def _depth_cb(self, msg: Image) -> None:
        if self._collecting:
            self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self._depth_frame = msg.header.frame_id

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            self._intrinsics = Intrinsics(
                fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5]
            )

    def _detections_cb(self, msg: ObjectDetectionArray) -> None:
        if not self._collecting or self._depth is None:
            return
        samples, rejections = self._samples, self._rejections
        for detection in msg.detections:
            if not self._matches(detection.label_text.lower()):
                continue
            try:
                samples.append(self._sample(detection))
            except GraspRejected as reason:
                rejections[str(reason)] += 1

    # Peak targets (clothes) are never detected directly: look inside containers.
    def _matches(self, label: str) -> bool:
        if self._known == GRASP_CLASS_PEAK:
            return OBJECT_GRASP_CLASS.get(label) == GRASP_CLASS_RIM
        return label == self._target

    # One detection in one frame -> one grasp sample (or GraspRejected).
    # Unknown objects whose measured recipe fails get the generic solid grasp.
    def _sample(self, detection: ObjectDetection) -> Sample:
        scene = self._scene(detection)
        grasp_class, measured = self._classify(scene)
        try:
            fit, grasps = grasp_for(scene, grasp_class)
        except GraspRejected:
            if self._known:
                raise
            grasp_class = GRASP_CLASS_GENERIC
            fit, grasps = grasp_for(scene, grasp_class)
        self._debug_pub.publish(self._pose(grasps[0].position, grasps[0].orientation))
        return Sample(grasp_class, measured, fit, grasps)

    # Returns (class to use, class geometry measured). Known names always win;
    # geometry is still measured so the logs show whether it would have agreed.
    def _classify(self, scene: Scene) -> tuple:
        if self._known is None:
            measured = classify(describe(scene))
            return measured, measured
        try:
            return self._known, classify(describe(scene))
        except GraspRejected:
            return self._known, None

    def _scene(self, detection: ObjectDetection) -> Scene:
        depth = self._depth  # local copy: the callback may swap the frame
        return Scene(
            depth=depth,
            intrinsics=self._intrinsics,
            cam_to_base=self._to_base(self._depth_frame),
            gripper_to_base=self._to_base(GRASP_LINK_FRAME),
            bbox=parse_bbox(
                detection.xmin,
                detection.ymin,
                detection.xmax,
                detection.ymax,
                depth.shape,
            ),
        )

    # Latest frame -> link_base transform as a 4x4 matrix.
    def _to_base(self, frame: str) -> np.ndarray:
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_FRAME,
                frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            ).transform
        except TransformException as error:
            raise GraspRejected(f"no TF {frame} -> {BASE_FRAME}") from error
        r, t = transform.rotation, transform.translation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def _pose(self, position, orientation) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        p, q = pose.pose.position, pose.pose.orientation
        p.x, p.y, p.z = map(float, position)
        q.x, q.y, q.z, q.w = map(float, orientation)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = GraspGenerator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
