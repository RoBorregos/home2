#!/usr/bin/env python3
"""Ground-truth object detector: serves the real detection interfaces from Gazebo poses.

An object is reported when its centre projects inside the camera image, is closer than
max_depth and is not hidden in the rendered depth image. Seeded misses and mislabels
come from the scenario so a run stays reproducible.
"""

import random

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from frida_constants.manipulation_constants import TRASH_BIN_NAME
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DETECTION_HANDLER_TOPIC_SRV,
    DETECTIONS_TOPIC,
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DetectionHandler
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from frida_task_sim import catalog, scenario as scenarios, venue as venues
from frida_task_sim.gz import PoseStream
from frida_task_sim.world import WORLD_NAME

ROBOT_MODEL = "frida"


class GtObjectDetector(Node):
    def __init__(self):
        super().__init__("gt_object_detector")
        scenario = scenarios.load(self.declare_parameter("scenario_file", "").value)
        self.max_depth = self.declare_parameter("max_depth", 2.5).value
        self.min_depth = self.declare_parameter("min_depth", 0.25).value
        self.objects = {o["id"]: o["name"] for o in scenario["objects"]}
        self.noise = scenario["detector"]
        furniture = venues.load(scenario["venue"])["furniture"].values()
        # Bins are static furniture, so their pose comes from the venue instead of Gazebo
        self.bins = [
            (np.array([*f["center"], f["height"] / 2]), f["height"], f["radius"])
            for f in furniture
            if f["shape"] == "bin"
        ]
        self.rng = random.Random(self.noise["seed"])
        self.poses = PoseStream(WORLD_NAME, names=[*self.objects, ROBOT_MODEL])
        self.bridge = CvBridge()
        self.info = None
        self.depth = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        group = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_info, 10)
        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._on_depth, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(ObjectDetectionArray, DETECTIONS_TOPIC, 10)
        self.create_service(
            DetectionHandler,
            DETECTION_HANDLER_TOPIC_SRV,
            self._handle,
            callback_group=group,
        )
        self.create_timer(0.5, self._publish, callback_group=group)
        self.get_logger().info(
            f"Ground-truth detector ready for {len(self.objects)} objects"
        )

    def _on_info(self, msg: CameraInfo):
        self.info = msg

    def _on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _camera_from_world(self):
        """(R, t) mapping world points into the camera optical frame."""
        base = self.poses.get(ROBOT_MODEL)
        if base is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                CAMERA_FRAME, "base_link", Time()
            ).transform
        except tf2_ros.TransformException:
            return None
        q, t = tf.rotation, tf.translation
        cam_from_base = Rotation.from_quat([q.x, q.y, q.z, q.w])
        base_position, base_rotation = base
        rotation = cam_from_base * base_rotation.inv()
        translation = np.array([t.x, t.y, t.z]) - rotation.apply(base_position)
        return rotation, translation

    def detect(self, noisy: bool = True) -> list[ObjectDetection]:
        camera = self._camera_from_world()
        if camera is None or self.info is None:
            return []
        rotation, translation = camera
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        width, height = self.info.width, self.info.height
        stamp = self.get_clock().now().to_msg()
        targets = [(TRASH_BIN_NAME, *b) for b in self.bins]
        for model, name in self.objects.items():
            pose = self.poses.get(model)
            if pose is None:
                continue
            position, orientation = pose
            h, r = catalog.height(name), catalog.footprint(name)
            targets.append(
                (name, position + orientation.apply([0.0, 0.0, h / 2]), h, r)
            )
        detections = []
        for name, centre, h, r in targets:
            x, y, z = rotation.apply(centre) + translation
            if not self.min_depth < z < self.max_depth:
                continue
            u, v = fx * x / z + cx, fy * y / z + cy
            if not (0 <= u < width and 0 <= v < height):
                continue
            point = np.array([x, y, z])
            if self.depth is not None:
                seen = float(self.depth[int(v), int(u)])
                if np.isfinite(seen) and seen > 0.0:
                    # Something nearer than the object's front surface hides it
                    if seen < z - r - 0.06:
                        continue
                    if abs(seen - z) < r + 0.06:
                        point *= seen / z
            if noisy and self.rng.random() < self.noise["miss_rate"]:
                continue
            label = name
            if noisy and self.rng.random() < self.noise["mislabel_rate"]:
                label = self.rng.choice(sorted(n for n in catalog.OBJECTS if n != name))
            half_w, half_h = fx * r / z, fy * max(h, 2 * r) / (2 * z)
            det = ObjectDetection()
            det.label_text = label
            det.score = 0.9
            det.xmin = float(np.clip((u - half_w) / width, 0, 1))
            det.xmax = float(np.clip((u + half_w) / width, 0, 1))
            det.ymin = float(np.clip((v - half_h) / height, 0, 1))
            det.ymax = float(np.clip((v + half_h) / height, 0, 1))
            det.point3d.header.frame_id = CAMERA_FRAME
            det.point3d.header.stamp = stamp
            det.point3d.point.x, det.point3d.point.y, det.point3d.point.z = map(
                float, point
            )
            detections.append(det)
        return detections

    def _publish(self):
        msg = ObjectDetectionArray(detections=self.detect(noisy=False))
        msg.header.frame_id = CAMERA_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _handle(self, request, response):
        detections = self.detect()
        if request.label and request.label != "all":
            detections = [d for d in detections if d.label_text == request.label]
        elif request.labels:
            detections = [d for d in detections if d.label_text in set(request.labels)]
        if request.closest_object and detections:
            p = lambda d: d.point3d.point  # noqa: E731
            detections = [
                min(detections, key=lambda d: p(d).x ** 2 + p(d).y ** 2 + p(d).z ** 2)
            ]
        response.detection_array.detections = detections
        response.success = len(detections) > 0
        labels = [d.label_text for d in detections]
        self.get_logger().info(f"DetectionHandler label='{request.label}' -> {labels}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GtObjectDetector()
    executor = MultiThreadedExecutor(num_threads=2)
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
