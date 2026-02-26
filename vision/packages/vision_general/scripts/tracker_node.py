#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2
import time
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

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
import copy

from vision_general.utils.reid_model import (
    load_network,
    compare_images,
    extract_feature_from_img,
    get_structure,
)

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

CONF_THRESHOLD = 0.8
DEPTH_THRESHOLD = 100


class SingleTracker(Node):
    def __init__(self):
        super().__init__("tracker_node")
        print("/???????????????????????????????_________________")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
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
            "forward": None,
            "backward": None,
            "left": None,
            "right": None,
            "coordinates": [],
        }
        self.depth_image_time = None
        pbar = tqdm.tqdm(total=1, desc="Loading models")

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

        self.output_image = []
        self.depth_image = []
        self.customer_image = []

        pbar.close()
        self.get_logger().info("Single Tracker Ready")

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
        # request = Trigger.Request()
        response = Trigger.Response()
        response.success = self.is_tracking_result
        if self.is_tracking_result:
            self.get_logger().info("Tracking")
        else:
<<<<<<< HEAD
            self.get_logger().info("Not tracking")
=======
            self.get_logger().info("Not racking")
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
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
            # if self.verbose:
            #     cv2.imshow("Tracking", self.output_image)
            #     if cv2.waitKey(1) & 0xFF == ord("q"):
            #         cv2.destroyAllWindows()
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

        if len(self.customer_image) != 0:
            h, w = self.customer_image.shape[:2]

            # 3. Determine square size (max of h or w)
            square_size = max(h, w)

            # 4. Create a black square image
            square_img = cv2.copyMakeBorder(
                self.customer_image,
                top=(square_size - h) // 2,
                bottom=(square_size - h + 1) // 2,  # Handles odd differences
                left=(square_size - w) // 2,
                right=(square_size - w + 1) // 2,
                borderType=cv2.BORDER_CONSTANT,
                value=(0, 0, 0),  # Black padding (BGR)
            )
            self.customer_publisher.publish(
                self.bridge.cv2_to_imgmsg(square_img, "bgr8")
            )

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def is_waving(self, pose, track_by):
        print("TRACKKK", track_by)
        if track_by == "wavingCustomer":
            if pose == "raising_left_arm" or pose == "raising_left_arm":
                return True
        return False

    def set_target(self, track_by="largest_person", value=""):
        """Set the target to track (Default: Largest person in frame)"""
        if self.image is None:
            self.get_logger().warn("No image available")
            self.is_tracking_result = False
            return False

        self.get_logger().info(f"Setting target by {track_by} with value {value}")
        self.person_data["id"] = None
        self.person_data["embeddings"] = None

        self.frame = copy.deepcopy(self.image)

        self.output_image = self.frame.copy()
        results = copy.deepcopy(self.results)

        largest_person = {
            "id": None,
            "area": 0,
            "bbox": None,
        }
        response_clean = ""
        # Check each detection
        for out in results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                # Get class name
                try:
                    track_id = box.id[0].item()
                except Exception:
                    track_id = -1

                # Get confidence
                prob = round(box.conf[0].item(), 2)

                if prob < CONF_THRESHOLD:
                    continue

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
                        print("POSEEEE", response_clean)

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
                            print("IS RAISING HANDDDDDDDDDDDDDDDDDD")
                            response_clean = value

                    if response_clean == value or response_clean == "1":
                        if pose.value == value:
                            self.success(f"Target found by {track_by}: {pose.value}")
                        elif response_clean == value:
                            self.success(
                                f"Target found by {track_by}: {response_clean}"
                            )
                        elif response_clean == "1":
                            self.success(f"Target found by {track_by}: {value}")
                        largest_person["id"] = track_id
                        largest_person["area"] = area
                        largest_person["bbox"] = (x1, y1, x2, y2)
                        # cv2.im(cropped_image)
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
            print("Waiting for future to complete...")
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

        if True:  # self.target_set:
            self.frame = self.image
            # image_time = copy.deepcopy(self.image_time)
            # depth_image_time = copy.deepcopy(self.depth_image_time)
            if self.frame is None:
                self.get_logger().error("No image available")
                return

            # if self.target_set:
            #     self.frame = self.image

            self.output_image = self.frame.copy()

            if self.frame is None:
                return

            self.output_image = self.frame.copy()

            self.results = self.model.track(
                self.frame,
                persist=True,
                tracker="bytetrack.yaml",
                classes=0,
                verbose=False,
            )

            if self.person_data["id"] is None:
                # self.frame = None
                return

            person_in_frame = False

            people = []

            # Check each detection
            for out in self.results:
                for box in out.boxes:
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                    # Get class name
                    class_id = box.cls[0].item()
                    label = self.model.names[class_id]

                    # id
                    try:
                        track_id = box.id[0].item()
                    except Exception as e:
                        print("Track id exception: ", e)
                        track_id = -1

                    # Get confidence
                    prob = round(box.conf[0].item(), 2)

                    if prob < CONF_THRESHOLD:
                        continue

                    people.append(
                        {
                            "track_id": track_id,
                            "x1": x1,
                            "y1": y1,
                            "x2": x2,
                            "y2": y2,
                        }
                    )

                    angle = None

                    # Check if person is in frame:
                    if track_id == self.person_data["id"]:
                        person_in_frame = True
                        self.person_data["coordinates"] = (x1, y1, x2, y2)
                        cv2.rectangle(
                            self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2
                        )
                        cropped_image = self.frame[y1:y2, x1:x2]
                        embedding = None

                        if self.person_data["embeddings"] is None:
                            pil_image = PILImage.fromarray(cropped_image)

                            with torch.no_grad():
                                embedding = extract_feature_from_img(
                                    pil_image, self.model_reid
                                )

                            self.person_data["embeddings"] = embedding

                        angle = self.pose_detection.personAngle(cropped_image)
                        if angle is not None and self.person_data[angle] is None:
                            print("Added angle: ", angle)
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
                        f"{label} {track_id}, Prob: {prob}, Angle: {angle}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                    )

            if not person_in_frame and len(people) > 0:
                for person in people:
                    cropped_image = self.frame[
                        person["y1"] : person["y2"], person["x1"] : person["x2"]
                    ]
                    pil_image = PILImage.fromarray(cropped_image)
                    with torch.no_grad():
                        embedding = extract_feature_from_img(pil_image, self.model_reid)
                    person_angle = self.pose_detection.personAngle(cropped_image)

                    if person_angle is not None:
                        if self.person_data[person_angle] is not None:
                            if compare_images(
                                embedding,
                                self.person_data[person_angle],
                                threshold=0.75,
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
                            if compare_images(
                                embedding,
                                self.person_data["embeddings"],
                                threshold=0.75,
                            ):
                                self.person_data["id"] = person["track_id"]
                                self.success(
                                    f"Person re-identified: {person['track_id']} without angle"
                                )
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
                    # self.get_logger().info(f"frame_shape: {self.frame.shape[1]} Point2D: {point2D[1]} normalized_point2D: {point2D_x_coord_normalized}")
                    self.centroid_publisher.publish(point2Dpoint)
                    depth = get_depth(self.depth_image, point2D)
                    point_2d_temp = (point2D[1], point2D[0])
                    point3D = deproject_pixel_to_point(
                        self.imageInfo, point_2d_temp, depth
                    )
                    # print(point3D)
                    point3D = float(point3D[0]), float(point3D[1]), float(point3D[2])
                    coords.point.x = point3D[0]
                    coords.point.y = point3D[1]
                    coords.point.z = point3D[2]
                    # self.point_pub.publish(coords)
                    self.results_publisher.publish(coords)
                # else:
                # self.get_logger().warn("Depth image not available")
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
