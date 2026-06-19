#!/usr/bin/env python3
"""
Node to track a single person and re-id them if necessary.
Requires 2 terminals minimum (3 if using pose/color detection via moondream).

--- Terminal 1: Run the tracker node ---
    ros2 run vision_general tracker_node

--- Terminal 2: Call a tracking service ---

    Option 1) Track the largest person (default):
        ros2 service call /vision/set_tracking_target std_srvs/srv/SetBool "{data: true}"

    Option 2) Track by gesture (e.g. waving):
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'gestures', value: 'waving'}"

    Option 3) Track by pose (standing, sitting, lying down):
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'poses', value: 'standing'}"

    Option 4) Track by clothing color:
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'color', value: 'red shirt'}"

    Disable tracking:
        ros2 service call /vision/set_tracking_target std_srvs/srv/SetBool "{data: false}"

    Check if tracking is active:
        ros2 service call /vision/is_tracking std_srvs/srv/Trigger

--- Terminal 3 (Orin only): Run the zed initialization
    zed

--- Terminal 3 (optional): Moondream vision model (required for poses/color) ---
    ros2 run vision_general moondream_node
"""

import copy
import time

import cv2
import rclpy
import torch
import tqdm
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    CENTROID_TOIC,
    CROP_QUERY_TOPIC,
    DEPTH_IMAGE_TOPIC,
    IS_TRACKING_TOPIC,
    RESULTS_TOPIC,
    SET_TARGET_BY_TOPIC,
    SET_TARGET_TOPIC,
    TRACKER_IMAGE_TOPIC,
)
from frida_constants.vision_enums import DetectBy
from frida_interfaces.srv import CropQuery, TrackBy
from vision_general.utils.calculations import (
    deproject_pixel_to_point,
    get2DCentroid,
    get_depth,
)
from vision_general.utils.ros_utils import wait_for_future
from models.tracker import TrackerModel, DEEPSORT_N_INIT

REID_EXTRACT_FREQ = 0.3
MAX_EMBEDDINGS = 128


class SingleTracker(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos,
            callback_group=self.image_callback_group,
        )
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )
        self.create_subscription(
            Bool, "/vision/tracker/active", self._active_callback, 10
        )

        self.create_service(SetBool, SET_TARGET_TOPIC, self.set_target_callback)
        self.create_service(TrackBy, SET_TARGET_BY_TOPIC, self.set_target_by_callback)
        self.create_service(Trigger, IS_TRACKING_TOPIC, self.get_is_tracking_callback)

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)
        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)
        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOIC, 10)
        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.last_reid_extraction = time.time()
        self.run_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.publish_callback_group = (
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.create_timer(0.1, self.run, callback_group=self.run_callback_group)
        self.create_timer(
            0.1, self.publish_image, callback_group=self.publish_callback_group
        )

    def setup(self):
        self.active = True
        self.target_set = False
        self.is_tracking_result = False
        self.image = None
        self.image_time = None
        self.output_image = []
        self.depth_image = []
        self.depth_image_time = None
        self.frame_id = "zed_left_camera_optical_frame"
        self._init_person_data()

        pbar = tqdm.tqdm(total=4, desc="Loading models")
        self.model = TrackerModel()
        self.model.load()
        pbar.update(4)
        pbar.close()
        self.get_logger().info("Single Tracker Ready (DeepSORT)")

    def _init_person_data(self):
        self.person_data = {
            "id": None,
            "embeddings": None,
            "num_embeddings": 0,
            "forward": None,
            "backward": None,
            "left": None,
            "right": None,
            "coordinates": [],
        }

    # ── ROS callbacks ──

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image_time = data.header.stamp

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image_time = data.header.stamp
        except Exception as e:
            self.get_logger().warn(f"depth_callback: {e}")

    def image_info_callback(self, data):
        self.imageInfo = data

    def _active_callback(self, msg):
        if msg.data == self.active:
            return
        self.active = msg.data
        self.get_logger().info(f"Tracker active: {self.active}")

    def get_is_tracking_callback(self, request, response):
        response = Trigger.Response()
        response.success = self.is_tracking_result
        self.get_logger().info(
            "Tracking" if self.is_tracking_result else "Not tracking"
        )
        return response

    def set_target_callback(self, request, response):
        self.target_set = request.data
        if self.target_set:
            response.success = self.set_target()
            self.get_logger().info("Tracking enabled: Target set")
        else:
            response.success = True
            self.get_logger().info("Tracking disabled")
        return response

    def set_target_by_callback(self, request, response):
        self.target_set = request.track_enabled
        if self.target_set:
            response.success = self.set_target(request.track_by, request.value)
            self.get_logger().info(
                f"Tracking enabled: Target set by {request.track_by}"
            )
        else:
            response.success = True
            self.get_logger().info("Tracking disabled")
        return response

    def publish_image(self):
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def success(self, message) -> None:
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    # ── Target selection ──

    def set_target(self, track_by="largest_person", value=""):
        if self.image is None:
            self.get_logger().warn("No image available")
            self.is_tracking_result = False
            return False

        self.get_logger().info(f"Setting target by {track_by} with value {value}")
        self.person_data["id"] = None
        self.person_data["embeddings"] = None
        self.person_data["num_embeddings"] = 0

        self.frame = copy.deepcopy(self.image)
        output_image = self.frame.copy()

        tracked_people = []
        for _ in range(DEEPSORT_N_INIT + 1):
            self.frame = copy.deepcopy(self.image)
            yolo_results = self.model.predict(self.frame)
            tracked_people = self.model.run_deepsort(self.frame, yolo_results)

        largest_person = {"id": None, "area": 0, "bbox": None}

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]
            cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            area = (x2 - x1) * (y2 - y1)

            if track_by == "largest_person":
                if area > largest_person["area"]:
                    largest_person = {
                        "id": track_id,
                        "area": area,
                        "bbox": (x1, y1, x2, y2),
                    }
                continue

            cropped = self.frame[y1:y2, x1:x2]
            response_clean = self._query_person(
                track_by, value, cropped, x1, y1, x2, y2
            )

            if response_clean == value or response_clean == "1":
                self.success(f"Target found by {track_by}: {response_clean}")
                largest_person = {
                    "id": track_id,
                    "area": area,
                    "bbox": (x1, y1, x2, y2),
                }

        if largest_person["id"] is not None:
            self.person_data["id"] = largest_person["id"]
            self.success(f"Target set: {largest_person['id']}")
            cv2.rectangle(
                output_image,
                largest_person["bbox"][:2],
                largest_person["bbox"][2:],
                (0, 255, 0),
                2,
            )
            cv2.putText(
                output_image,
                f"Target by {track_by}: {largest_person['id']}",
                largest_person["bbox"][:2],
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            self.output_image = output_image
            return True

        self.get_logger().warn("No person found")
        return False

    def _query_person(self, track_by: str, value: str, crop, x1, y1, x2, y2) -> str:
        """Dispatch a single person through gesture/pose/color detection. Returns response string."""
        if track_by == DetectBy.GESTURES.value:
            pose = self.model.pose.detectGesture(crop)
            response = pose.value
            if value == "wavingCustomer" and response != value:
                pts, kpc = self.model.pose._get_keypoints(crop)
                if self.model.pose.is_waving_from_keypoints(pts, kpc):
                    response = value
            return response

        if track_by == DetectBy.POSES.value:
            prompt = (
                "Respond 'standing' if the person in the image is standing, "
                "'sitting' if the person in the image is sitting, "
                "'lying down' if the person in the image is lying down or "
                "'unknown' if the person is not doing any of the previous."
            )
            ok, result = self.moondream_crop_query(
                prompt, [float(y1), float(x1), float(y2), float(x2)]
            )
            if ok:
                return result.replace(" ", "_").lstrip("_")
            return ""

        if track_by == DetectBy.COLOR.value:
            prompt = f"Reply only with 1 if the person is wearing {value}. Otherwise, reply only with 0."
            ok, result = self.moondream_crop_query(
                prompt, [float(y1), float(x1), float(y2), float(x2)]
            )
            if ok:
                return result.strip()
            return ""

        return ""

    def moondream_crop_query(self, prompt: str, bbox: list[float]) -> tuple[int, str]:
        self.get_logger().info(f"Querying image with prompt: {prompt}")
        h, w = self.image.shape[:2]
        req = CropQuery.Request()
        req.query = prompt
        req.ymin = bbox[0] / h
        req.xmin = bbox[1] / w
        req.ymax = bbox[2] / h
        req.xmax = bbox[3] / w
        future = wait_for_future(self.moondream_client.call_async(req), 15)
        result = future.result()
        if result is None:
            self.get_logger().error("Moondream service returned None.")
            return 0, "0"
        if result.success:
            self.get_logger().info(f"Moondream result: {result.result}")
            return 1, result.result
        return 0, "0"

    # ── Main tracking loop ──

    def run(self):
        if not self.active or not self.target_set:
            self.is_tracking_result = False
            return

        self.frame = copy.deepcopy(self.image)
        if self.frame is None:
            self.get_logger().error("No image available")
            return

        output_image = self.frame.copy()

        yolo_results = self.model.predict(self.frame)
        tracked_people = self.model.run_deepsort(self.frame, yolo_results)

        if self.person_data["id"] is None:
            self.frame = None
            return

        person_in_frame = False

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]
            angle = None

            if track_id == self.person_data["id"]:
                person_in_frame = True
                self.person_data["coordinates"] = (x1, y1, x2, y2)
                cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cropped = self.frame[y1:y2, x1:x2]
                embedding = None

                if self.person_data[
                    "embeddings"
                ] is None or time.time() - self.last_reid_extraction > (
                    1 / REID_EXTRACT_FREQ
                ):
                    self.last_reid_extraction = time.time()
                    embedding = self.model.extract_reid_tensor(cropped)
                    if self.person_data["embeddings"] is None:
                        self.person_data["embeddings"] = torch.zeros(
                            (MAX_EMBEDDINGS, embedding.shape[1]),
                            device="cuda" if torch.cuda.is_available() else "cpu",
                        )
                        self.person_data["embeddings"][
                            self.person_data["num_embeddings"]
                        ] = embedding.squeeze()
                        self.person_data["num_embeddings"] += 1
                    elif (
                        not self.model.compare_batch(
                            embedding, self.person_data["embeddings"]
                        )
                        and self.person_data["num_embeddings"] < MAX_EMBEDDINGS
                    ):
                        self.person_data["embeddings"][
                            self.person_data["num_embeddings"]
                        ] = embedding.squeeze()
                        self.person_data["num_embeddings"] += 1

                angle = self.model.pose.personAngle(cropped)
                if angle is not None and self.person_data[angle] is None:
                    if embedding is None:
                        embedding = self.model.extract_reid_tensor(cropped)
                    self.person_data[angle] = embedding
            else:
                cv2.rectangle(output_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

            cv2.putText(
                output_image,
                f"person {track_id}, Angle: {angle}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )
            self.output_image = output_image

        # Re-identification if target lost
        reid_start = None
        if not person_in_frame and tracked_people:
            reid_start = time.time()
            crops = [
                self.frame[p["y1"] : p["y2"], p["x1"] : p["x2"]] for p in tracked_people
            ]
            embeddings_batch = self.model.extract_reid_batch(crops)

            for i, person in enumerate(tracked_people):
                crop = crops[i]
                emb = embeddings_batch[i]
                person_angle = self.model.pose.personAngle(crop)

                if person_angle is not None:
                    if self.person_data[
                        person_angle
                    ] is not None and self.model.compare(
                        emb, self.person_data[person_angle]
                    ):
                        self._reidentify(person)
                        person_in_frame = True
                        break
                elif (
                    self.person_data["embeddings"] is not None
                    and self.person_data["num_embeddings"] > 0
                ):
                    if self.model.compare_batch(emb, self.person_data["embeddings"]):
                        self._reidentify(person)
                        person_in_frame = True
                        break

        if person_in_frame:
            if reid_start:
                self.get_logger().info(f"ReID took {time.time()-reid_start:.2f}s")
            self.is_tracking_result = True
            self._publish_position()
        else:
            self.is_tracking_result = False

        self.frame = None

    def _reidentify(self, person):
        self._init_person_data()
        self.person_data["id"] = person["track_id"]
        self.person_data["coordinates"] = (
            person["x1"],
            person["y1"],
            person["x2"],
            person["y2"],
        )
        self.success(f"Person re-identified: {person['track_id']}")

    def _publish_position(self):
        if len(self.depth_image) == 0:
            self.get_logger().warn("Depth image not available")
            return

        coords = PointStamped()
        coords.header.frame_id = self.frame_id
        coords.header.stamp = self.depth_image_time
        point2D = get2DCentroid(self.person_data["coordinates"], self.depth_image)
        point2D_x = float(point2D[1])
        centroid_pt = Point()
        centroid_pt.x = (point2D_x / (self.frame.shape[1] / 2)) - 1
        centroid_pt.y = 0.0
        centroid_pt.z = 0.0
        self.centroid_publisher.publish(centroid_pt)
        depth = get_depth(self.depth_image, point2D)
        point3D = deproject_pixel_to_point(
            self.imageInfo, (point2D[1], point2D[0]), depth
        )
        coords.point.x, coords.point.y, coords.point.z = (
            float(point3D[0]),
            float(point3D[1]),
            float(point3D[2]),
        )
        self.results_publisher.publish(coords)


def main(args=None):
    rclpy.init(args=args)
    node = SingleTracker()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
