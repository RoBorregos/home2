#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2
import time
import numpy as np
from ultralytics import YOLO
from PIL import Image as PILImage
import tqdm
import torch.nn as nn
import torch
from vision_general.utils.calculations import (
    get2DCentroid,
    get_depth,
    deproject_pixel_to_point,
)

import copy
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from vision_general.utils.reid_model import (
    load_network,
    compare_images,
    compare_images_batch,
    extract_feature_from_img,
    extract_feature_from_img_batch,
    get_structure,
)

from vision_general.utils.deep_sort.tracker import Tracker as DeepSORTTracker
from vision_general.utils.deep_sort.detection import Detection as DeepSORTDetection
from vision_general.utils.deep_sort.nn_matching import NearestNeighborDistanceMetric

from std_srvs.srv import SetBool, Trigger
from frida_interfaces.srv import TrackBy, CropQuery
from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SET_TARGET_TOPIC,
    SET_TARGET_BY_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOIC,
    CROP_QUERY_TOPIC,
    IS_TRACKING_TOPIC,
    CUSTOMER,
)
from frida_constants.vision_enums import DetectBy

CONF_THRESHOLD = 0.6
DEPTH_THRESHOLD = 100
REID_EXTRACT_FREQ = 0.3
MAX_EMBEDDINGS = 128
DEEPSORT_MAX_COSINE_DISTANCE = 0.3
DEEPSORT_NN_BUDGET = 100
DEEPSORT_MAX_AGE = 30
DEEPSORT_N_INIT = 3


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

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )

        self.set_target_service = self.create_service(
            SetBool, SET_TARGET_TOPIC, self.set_target_callback
        )

        self.set_target_by_service = self.create_service(
            TrackBy, SET_TARGET_BY_TOPIC, self.set_target_by_callback
        )

        self.get_is_tracking_service = self.create_service(
            Trigger, IS_TRACKING_TOPIC, self.get_is_tracking_callback
        )

        self.is_tracking_result = False

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)

        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)

        self.customer_publisher = self.create_publisher(Image, CUSTOMER, 10)

        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOIC, 10)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.last_reid_extraction = time.time()
        self.create_timer(0.1, self.run)
        self.create_timer(0.01, self.publish_image)

    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        self.image = None
        self.image_time = None
        self.frame_id = "zed_left_camera_optical_frame"
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
        self.depth_image_time = None
        pbar = tqdm.tqdm(total=4, desc="Loading models")

        self.model = YOLO("yolov8n.pt")
        self.pose_detection = PoseDetection()

        # Load the ReID model
        structure = get_structure()
        pbar.update(1)
        self.model_reid = load_network(structure)
        pbar.update(1)
        self.model_reid.classifier.classifier = nn.Sequential()
        pbar.update(1)
        use_gpu = torch.cuda.is_available()
        if use_gpu:
            self.model_reid = self.model_reid.cuda()
        pbar.update(1)

        # Initialize DeepSORT tracker
        metric = NearestNeighborDistanceMetric(
            "cosine", DEEPSORT_MAX_COSINE_DISTANCE, DEEPSORT_NN_BUDGET
        )
        self.deepsort_tracker = DeepSORTTracker(
            metric, max_age=DEEPSORT_MAX_AGE, n_init=DEEPSORT_N_INIT
        )

        self.output_image = []
        self.depth_image = []
        self.customer_image = []

        pbar.close()
        self.get_logger().info("Single Tracker Ready (DeepSORT)")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image_time = data.header.stamp

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
            self.depth_image_time = data.header.stamp
        except Exception as e:
            print(f"Error: {e}")

    def get_is_tracking_callback(self, request, response):
        response = Trigger.Response()
        response.success = self.is_tracking_result
        if self.is_tracking_result:
            self.get_logger().info("Tracking")
        else:
            self.get_logger().info("Not tracking")
        return response

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def set_target_callback(self, request, response):
        """Callback to set the target to track"""
        self.target_set = request.data
        if self.target_set:
            response.success = self.set_target()
            self.get_logger().info("Tracking enabled: Target set")
        else:
            response.success = True
            self.get_logger().info("Tracking disabled")
        return response

    def set_target_by_callback(self, request, response):
        """Callback to set target by pose, gesture, clothes, etc"""
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
        """Publish the image to the camera topic"""
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

        if len(self.customer_image) != 0:
            h, w = self.customer_image.shape[:2]
            square_size = max(h, w)
            square_img = cv2.copyMakeBorder(
                self.customer_image,
                top=(square_size - h) // 2,
                bottom=(square_size - h + 1) // 2,
                left=(square_size - w) // 2,
                right=(square_size - w + 1) // 2,
                borderType=cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )
            self.customer_publisher.publish(
                self.bridge.cv2_to_imgmsg(square_img, "bgr8")
            )

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def _extract_reid_feature_np(self, cropped_image):
        """Extract ReID feature as numpy array for DeepSORT."""
        pil_image = PILImage.fromarray(cropped_image)
        with torch.no_grad():
            embedding = extract_feature_from_img(pil_image, self.model_reid)
        return embedding.cpu().numpy().flatten()

    def _run_deepsort(self, frame, yolo_results):
        """Run DeepSORT on YOLO detections, returns list of confirmed tracks."""
        detections = []
        for out in yolo_results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                prob = round(box.conf[0].item(), 2)
                if prob < CONF_THRESHOLD:
                    continue
                cropped = frame[y1:y2, x1:x2]
                if cropped.size == 0:
                    continue
                feature = self._extract_reid_feature_np(cropped)
                w = x2 - x1
                h = y2 - y1
                det = DeepSORTDetection(
                    tlwh=np.array([x1, y1, w, h], dtype=np.float64),
                    confidence=prob,
                    feature=feature,
                )
                detections.append(det)

        self.deepsort_tracker.predict()
        self.deepsort_tracker.update(detections)

        tracks = []
        for track in self.deepsort_tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            tracks.append(
                {
                    "track_id": track.track_id,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            )
        return tracks

    def set_target(self, track_by="largest_person", value=""):
        """Set the target to track (Default: Largest person in frame)"""
        if self.image is None:
            self.get_logger().warn("No image available")
            self.is_tracking_result = False
            return False

        self.get_logger().info(f"Setting target by {track_by} with value {value}")
        self.person_data["id"] = None
        self.person_data["embeddings"] = None
        self.person_data["num_embeddings"] = 0

        self.frame = copy.deepcopy(self.image)
        self.output_image = self.frame.copy()

        # Run YOLO + DeepSORT to get tracked people
        yolo_results = self.model.track(
            self.frame,
            persist=True,
            tracker="bytetrack.yaml",
            classes=0,
            verbose=False,
        )
        tracked_people = self._run_deepsort(self.frame, yolo_results)

        largest_person = {
            "id": None,
            "area": 0,
            "bbox": None,
        }
        response_clean = ""

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]

            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

            area = (x2 - x1) * (y2 - y1)
            if track_by == "largest_person":
                if area > largest_person["area"]:
                    largest_person["id"] = track_id
                    largest_person["area"] = area
                    largest_person["bbox"] = (x1, y1, x2, y2)
            else:
                cropped_image = self.frame[y1:y2, x1:x2]

                if track_by == DetectBy.GESTURES.value:
                    self.get_logger().info(f"Detecting gesture {value} ")
                    pose = self.pose_detection.detectGesture(cropped_image)
                    response_clean = pose.value

                elif track_by == DetectBy.POSES.value:
                    prompt = "Respond 'standing' if the person in the image is standing, 'sitting' if the person in the image is sitting, 'lying down' if the person in the image is lying down or 'unknown' if the person is not doing any of the previous."
                    status, response_q = self.moondream_crop_query(
                        prompt, [float(y1), float(x1), float(y2), float(x2)]
                    )
                    if status:
                        self.get_logger().info(f"The person is {response_q}.")
                        response_clean = response_q.replace(" ", "_")
                        response_clean = response_clean.replace("_", "", 1)

                elif track_by == DetectBy.COLOR.value:
                    prompt = f"Reply only with 1 if the person is wearing {value}. Otherwise, reply only with 0."
                    status, response_q = self.moondream_crop_query(
                        prompt, [float(y1), float(x1), float(y2), float(x2)]
                    )
                    if status:
                        response_clean = response_q.strip()

                if value == "wavingCustomer":
                    raising = self.pose_detection.is_waving_customer(cropped_image)
                    if raising:
                        response_clean = value

                if response_clean == value or response_clean == "1":
                    self.success(f"Target found by {track_by}: {response_clean}")
                    largest_person["id"] = track_id
                    largest_person["area"] = area
                    largest_person["bbox"] = (x1, y1, x2, y2)
                    self.customer_image = copy.deepcopy(cropped_image)
                else:
                    self.get_logger().warn(
                        f"Person detected with {track_by}: {response_clean} but not {value}"
                    )

        if largest_person["id"] is not None:
            self.person_data["id"] = largest_person["id"]
            self.success(f"Target set: {largest_person['id']}")
            cv2.rectangle(
                self.output_image,
                largest_person["bbox"][:2],
                largest_person["bbox"][2:],
                (0, 255, 0),
                2,
            )
            cv2.putText(
                self.output_image,
                f"Target by {track_by}: {largest_person['id']}",
                largest_person["bbox"][:2],
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            return True
        else:
            self.get_logger().warn("No person found")
            return False

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            pass
        return future

    def moondream_crop_query(self, prompt: str, bbox: list[float]) -> tuple[int, str]:
        """Makes a query of the current image using moondream."""
        self.get_logger().info(f"Querying image with prompt: {prompt}")

        height, width = self.image.shape[:2]

        ymin = bbox[0] / height
        xmin = bbox[1] / width
        ymax = bbox[2] / height
        xmax = bbox[3] / width

        request = CropQuery.Request()
        request.query = prompt
        request.ymin = ymin
        request.xmin = xmin
        request.ymax = ymax
        request.xmax = xmax

        future = self.moondream_client.call_async(request)
        future = self.wait_for_future(future, 15)
        result = future.result()
        if result is None:
            self.get_logger().error("Moondream service returned None.")
            return 0, "0"
        if result.success:
            self.get_logger().info(f"Moondream result: {result.result}")
            return 1, result.result

    def run(self):
        """Main loop to run the tracker"""

        if not self.target_set:
            self.is_tracking_result = False
            return

        self.frame = copy.deepcopy(self.image)
        if self.frame is None:
            self.get_logger().error("No image available")
            return

        self.output_image = self.frame.copy()

        # Run YOLO detection
        start_time = time.time()
        yolo_results = self.model.track(
            self.frame,
            persist=True,
            tracker="bytetrack.yaml",
            classes=0,
            verbose=False,
        )
        self.get_logger().info(
            f"Det+Tracking took {time.time() - start_time:.2f} seconds"
        )

        # Run DeepSORT with ReID features
        tracked_people = self._run_deepsort(self.frame, yolo_results)

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
                cv2.rectangle(
                    self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2
                )
                cropped_image = self.frame[y1:y2, x1:x2]
                embedding = None
                pil_image = PILImage.fromarray(cropped_image)

                if self.person_data[
                    "embeddings"
                ] is None or time.time() - self.last_reid_extraction > (
                    1 / REID_EXTRACT_FREQ
                ):
                    self.last_reid_extraction = time.time()
                    with torch.no_grad():
                        embedding = extract_feature_from_img(
                            pil_image, self.model_reid
                        )
                    if self.person_data["embeddings"] is None:
                        self.person_data["embeddings"] = torch.zeros(
                            (MAX_EMBEDDINGS, embedding.shape[1]),
                            device="cuda" if torch.cuda.is_available() else "cpu",
                        )
                        self.person_data["embeddings"][
                            self.person_data["num_embeddings"]
                        ] = embedding.squeeze()
                        self.person_data["num_embeddings"] += 1
                    else:
                        embedding_exists = compare_images_batch(
                            embedding,
                            self.person_data["embeddings"],
                            threshold=0.7,
                        )
                        if (
                            not embedding_exists
                            and self.person_data["num_embeddings"] < MAX_EMBEDDINGS
                        ):
                            self.person_data["embeddings"][
                                self.person_data["num_embeddings"]
                            ] = embedding.squeeze()
                            self.person_data["num_embeddings"] += 1

                angle = self.pose_detection.personAngle(cropped_image)
                if angle is not None and self.person_data[angle] is None:
                    if embedding is None:
                        pil_image = PILImage.fromarray(cropped_image)
                        with torch.no_grad():
                            embedding = extract_feature_from_img(
                                pil_image, self.model_reid
                            )
                    self.person_data[angle] = embedding

            else:
                cv2.rectangle(
                    self.output_image, (x1, y1), (x2, y2), (255, 0, 0), 2
                )

            cv2.putText(
                self.output_image,
                f"person {track_id}, Angle: {angle}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )

        # Re-identification if person lost
        if not person_in_frame and len(tracked_people) > 0:
            img_list = []
            for person in tracked_people:
                cropped_image = self.frame[
                    person["y1"] : person["y2"], person["x1"] : person["x2"]
                ]
                pil_image = PILImage.fromarray(cropped_image)
                img_list.append(pil_image)

            with torch.no_grad():
                embeddings_batch = extract_feature_from_img_batch(
                    img_list, self.model_reid
                )

            for i, person in enumerate(tracked_people):
                cropped_image = self.frame[
                    person["y1"] : person["y2"], person["x1"] : person["x2"]
                ]
                embedding = embeddings_batch[i]
                person_angle = self.pose_detection.personAngle(cropped_image)

                if person_angle is not None:
                    if self.person_data[person_angle] is not None:
                        if compare_images(
                            embedding, self.person_data[person_angle], threshold=0.7
                        ):
                            self.person_data["id"] = person["track_id"]
                            self.person_data["coordinates"] = (
                                person["x1"],
                                person["y1"],
                                person["x2"],
                                person["y2"],
                            )
                            self.success(
                                f"Person re-identified: {person['track_id']} with angle {person_angle}"
                            )
                            person_in_frame = True
                            break
                else:
                    if (
                        self.person_data["embeddings"] is not None
                        and self.person_data["num_embeddings"] > 0
                    ):
                        embedding_exists = compare_images_batch(
                            embedding,
                            self.person_data["embeddings"],
                            threshold=0.7,
                        )
                        if embedding_exists:
                            self.success(
                                f"Person re-identified: {person['track_id']} without angle"
                            )
                            self.person_data["id"] = person["track_id"]
                            person_in_frame = True
                            break

        if person_in_frame:
            self.is_tracking_result = True
            if len(self.depth_image) > 0 and (
                (
                    self.depth_image_time.nanosec - self.image_time.nanosec
                    > -DEPTH_THRESHOLD
                )
                and (
                    self.depth_image_time.nanosec - self.image_time.nanosec
                    < DEPTH_THRESHOLD
                )
            ):
                coords = PointStamped()
                coords.header.frame_id = self.frame_id
                coords.header.stamp = self.depth_image_time
                point2D = get2DCentroid(
                    self.person_data["coordinates"], self.depth_image
                )
                point2D_x_coord = float(point2D[1])
                point2D_x_coord_normalized = (
                    point2D_x_coord / (self.frame.shape[1] / 2)
                ) - 1
                point2Dpoint = Point()
                point2Dpoint.x = float(point2D_x_coord_normalized)
                point2Dpoint.y = 0.0
                point2Dpoint.z = 0.0
                self.centroid_publisher.publish(point2Dpoint)
                depth = get_depth(self.depth_image, point2D)
                point_2d_temp = (point2D[1], point2D[0])
                point3D = deproject_pixel_to_point(
                    self.imageInfo, point_2d_temp, depth
                )
                point3D = float(point3D[0]), float(point3D[1]), float(point3D[2])
                coords.point.x = point3D[0]
                coords.point.y = point3D[1]
                coords.point.z = point3D[2]
                self.results_publisher.publish(coords)
            else:
                self.get_logger().warn("Depth image not available")
        else:
            self.is_tracking_result = False

        self.frame = None


def main(args=None):
    rclpy.init(args=args)
    node = SingleTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
