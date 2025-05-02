#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2
from ultralytics import YOLO
from PIL import Image as PILImage
import tqdm
import torch.nn as nn
import torch
import math as m
from std_msgs.msg import Header
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
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from vision_general.utils.reid_model import (
    load_network,
    compare_images,
    extract_feature_from_img,
    get_structure,
)

from std_srvs.srv import SetBool
from frida_interfaces.srv import TrackBy
from vision_general.pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SET_TARGET_TOPIC,
    SET_TARGET_BY_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOIC,
)
from frida_constants.vision_enums import DetectBy

CONF_THRESHOLD = 0.6


class SingleTracker(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.bridge = CvBridge()

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

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)

        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)

        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOIC, 10)

        self.verbose = self.declare_parameter("verbose", True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.setup()
        self.create_timer(0.1, self.run)
        self.create_timer(0.01, self.publish_image)
        
    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        self.image = None
        self.person_data = {
            "id": None,
            "embeddings": None,
            "forward": None,
            "backward": None,
            "left": None,
            "right": None,
            "coordinates": [],
        }

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

        pbar.close()
        self.get_logger().info("Single Tracker Ready")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
        except Exception as e:
            print(f"Error: {e}")

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

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def set_target(self, track_by="largest_person", value=""):
        """Set the target to track (Default: Largest person in frame)"""
        if self.image is None:
            self.get_logger().warn("No image available")
            return False

        self.get_logger().info(f"Setting target by {track_by} with value {value}")

        self.frame = self.image
        self.output_image = self.frame.copy()
        results = self.model.track(
            self.frame,
            persist=True,
            tracker="bytetrack.yaml",
            classes=0,
            verbose=False,
        )

        largest_person = {
            "id": None,
            "area": 0,
            "bbox": None,
        }

        # Check each detection
        for out in results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                # Get class name
                try:
                    track_id = 1
                except Exception as e:
                    print("Track id exception: ", e)
                    track_id = -1

                print(track_id)
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
                        pose = self.pose_detection.detectGesture(cropped_image)

                    elif track_by == DetectBy.POSES.value:
                        pose = self.pose_detection.detectPose(cropped_image)

                    if pose.value == value:
                        self.success(f"Target found by {track_by}: {pose.value}")
                        largest_person["id"] = track_id
                        largest_person["area"] = area
                        largest_person["bbox"] = (x1, y1, x2, y2)
                    else:
                        self.get_logger().warn(
                            f"Person detected with {track_by}: {pose.value}"
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

    def run(self):
        """Main loop to run the tracker"""
        if self.target_set:
            self.frame = self.image

            if self.frame is None or self.person_data["id"] is None:
                return

            self.output_image = self.frame.copy()

            results = self.model.track(
                self.frame,
                persist=True,
                tracker="bytetrack.yaml",
                classes=0,
                verbose=False,
            )

            person_in_frame = False

            people = []

            # Check each detection
            for out in results:
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
                            if compare_images(
                                embedding, self.person_data["embeddings"], threshold=0.7
                            ):
                                self.person_data["id"] = person["track_id"]
                                self.success(
                                    f"Person re-identified: {person['track_id']} without angle"
                                )
                                break
            if person_in_frame:
                if len(self.depth_image) > 0:
                    try:
                        point2D = get2DCentroid(self.person_data["coordinates"], self.frame)
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

                        point_3D = PointStamped(
                        header=Header(frame_id="zed_camera_link", stamp = self.get_clock().now().to_msg()),
                        point=Point(),
                        )
                        point_centroid = get2DCentroid(self.person_data["coordinates"], self.depth_image)
                        depth = get_depth(self.depth_image, point_centroid)
                        point_3D_ = deproject_pixel_to_point(self.imageInfo, point_centroid, depth)
                    
                        point_3D.point.x = float(point_3D_[0])
                        point_3D.point.y = float(point_3D_[1])
                        point_3D.point.z = float(point_3D_[2])

                        original_orientation = m.atan2(point_3D.point.y, point_3D.point.x)
                        original_distance = m.sqrt(point_3D.point.x ** 2 + point_3D.point.y ** 2)

                        # Transform the point
                        transformed_orientation = original_orientation + m.pi*(1.25)

                        point_3D.point.x = original_distance * m.cos(transformed_orientation)
                        point_3D.point.y = original_distance * m.sin(transformed_orientation)

                        transform = self.tf_buffer.lookup_transform(
                        'map',
                        point_3D.header.frame_id,
                        rclpy.time.Time(),
                        )   
                        transformed_point = do_transform_point(point_3D, transform)
                        
                        self.results_publisher.publish(transformed_point)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to transform point: {e}")
                else:
                    self.get_logger().warn("Depth image not available")


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
