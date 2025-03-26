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

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from vision_general.utils.reid_model import (
    load_network,
    compare_images,
    extract_feature_from_img,
    get_structure,
)

from std_srvs.srv import SetBool
from vision_general.pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SET_TARGET_TOPIC,
)

CONF_THRESHOLD = 0.6


class SingleTracker(Node):
    def __init__(self):
        super().__init__("single_tracker")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.set_target_service = self.create_service(
            SetBool, SET_TARGET_TOPIC, self.set_target_callback
        )

        self.setup()
        self.create_timer(0.1, self.run)

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

        pbar.close()
        self.get_logger().info("Single Tracker Ready")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

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

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def set_target(self):
        """Set the target to track (Default: Largest person in frame)"""
        self.frame = self.image
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

                area = (x2 - x1) * (y2 - y1)
                if area > largest_person["area"]:
                    largest_person["id"] = track_id
                    largest_person["area"] = area

        if largest_person["id"] is not None:
            self.person_data["id"] = largest_person["id"]
            self.success(f"Target set: {largest_person['id']}")
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
                        cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
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
                            if embedding is None:
                                pil_image = PILImage.fromarray(cropped_image)
                                with torch.no_grad():
                                    embedding = extract_feature_from_img(
                                        pil_image, self.model_reid
                                    )

                            self.person_data[angle] = embedding

                    else:
                        cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

                    cv2.putText(
                        self.frame,
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
                                self.success(
                                    f"Person re-identified: {person['track_id']} with angle {person_angle}"
                                )
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

            cv2.imshow("Tracking", self.frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()


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
