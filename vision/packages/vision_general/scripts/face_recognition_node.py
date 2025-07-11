#!/usr/bin/env python3

"""
Node to recognize known people and
name new faces.
- Publisher for largest face detected
coordinates for arm following.
- Service to assign name to face.
"""

import os
import pathlib

import cv2
import face_recognition
import numpy as np
import rclpy
import rclpy.duration
import tqdm
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FACE_RECOGNITION_IMAGE,
    FOLLOW_BY_TOPIC,
    FOLLOW_TOPIC,
    PERSON_LIST_TOPIC,
    PERSON_NAME_TOPIC,
    SAVE_NAME_TOPIC,
)
from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import SaveName

# from vision_general.utils.calculations import (
#     get_depth,
#     deproject_pixel_to_point,
# )


DEFAULT_NAME = "ale"
TRACK_THRESHOLD = 50
MATCH_THRESHOLD = 0.5
MAX_DEGREE = 1
RESIZE_FACTOR = 1

PATH = str(pathlib.Path(__file__).parent)
PATH = get_package_share_directory("vision_general")
KNOWN_FACES_PATH = PATH + "/Utils/known_faces"


class FaceRecognition(Node):
    def __init__(self):
        super().__init__("face_recognition")
        self.bridge = CvBridge()
        self.pbar = tqdm.tqdm(total=2)
        # self.callback_gorup = rclpy.callback_groups.ReentrantCallbackGroup()
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )
        self.new_name_service = self.create_service(
            SaveName, SAVE_NAME_TOPIC, self.new_name_callback
        )
        self.follow_by_service = self.create_service(
            SaveName, FOLLOW_BY_TOPIC, self.follow_by_name_callback
        )
        self.follow_publisher = self.create_publisher(Point, FOLLOW_TOPIC, 10)
        self.view_pub = self.create_publisher(Image, FACE_RECOGNITION_IMAGE, 10)
        self.name_publisher = self.create_publisher(String, PERSON_NAME_TOPIC, 10)
        self.person_list_publisher = self.create_publisher(
            PersonList, PERSON_LIST_TOPIC, 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.annotated_frame = []
        self.setup()
        self.create_timer(0.05, self.run)
        # self.create_timer(0.05, self.publish_image)

    def setup(self):
        """Setup face recognition, reset variables and load models"""
        random = face_recognition.load_image_file(f"{KNOWN_FACES_PATH}/random.png")
        random_encodings = face_recognition.face_encodings(random)[0]
        self.pbar.update(1)
        self.new_name = ""
        self.image_view = None
        self.image = None
        self.prev_faces = []
        self.curr_faces = []
        self.depth_image = []
        self.follow_name = "area"
        self.id = None
        self.processing_id = rclpy.duration.Infinite
        self.default_name = self.declare_parameter("default_name", DEFAULT_NAME)
        self.default_name = self.default_name.value

        self.people = [[random_encodings, "random"]]
        self.people_encodings = [random_encodings]
        self.people_names = ["random"]

        self.clear()
        self.process_imgs()
        self.get_logger().info("Face Recognition Ready")

    def image_callback(self, data):
        """Callback to get image from camera"""
        # self.get_logger().info("img received reentrant")
        self.id = data.header.stamp
        # self.get_logger().info(f"s{self.id}")
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

    def new_name_callback(self, req, res):
        """Callback to add a new face to the known faces"""
        self.get_logger().info("Executing service new face")
        self.new_name = req.name
        if len(self.curr_faces) == 0:
            self.get_logger().info("No face detected")
            res.success = False
        else:
            self.get_logger().info(f"New name: {self.new_name}")
            res.success = True
        return res

    def follow_by_name_callback(self, req, res):
        """Callback to follow face by name or area"""
        self.get_logger().info("Executing service follow by")
        self.follow_name = req.name
        if len(self.curr_faces) == 0:
            self.get_logger().info("No face detected")
            res.success = False
        else:
            self.get_logger().info(f"New name: {self.follow_name}")
            res.success = True
        return res

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self) -> None:
        """Publish image with annotations"""
        if len(self.annotated_frame) > 0:
            self.view_pub.publish(
                self.bridge.cv2_to_imgmsg(self.annotated_frame, "bgr8")
            )

    def process_imgs(self) -> None:
        """Make encodings of known people images"""
        self.get_logger().info("Processing images")
        for filename in os.listdir(KNOWN_FACES_PATH):
            if filename == ".DS_Store":
                continue

            self.process_img(filename)

    def clear(self) -> None:
        """Clear previous results"""
        for filename in os.listdir(KNOWN_FACES_PATH):
            if (
                filename == ".DS_Store"
                or filename == "random.png"
                or filename == self.default_name + ".png"
            ):
                continue

            file_path = os.path.join(KNOWN_FACES_PATH, filename)
            os.remove(file_path)

    def process_img(self, filename: str) -> None:
        """Process image, obtain encodings and add to known people"""
        img = face_recognition.load_image_file(f"{KNOWN_FACES_PATH}/{filename}")
        cur_encodings = face_recognition.face_encodings(img)

        if len(cur_encodings) == 0:
            print("no encodings found")
            return

        if len(cur_encodings) > 0:
            cur_encodings = cur_encodings[0]

        self.people_encodings.append(cur_encodings)
        self.people_names.append(filename[:-4])
        self.people.append([cur_encodings, filename[:-4]])

    def save_face(self, name: str, xc: float, yc: float) -> None:
        """Save face to list and return Person message"""
        self.curr_faces.append({"x": xc, "y": yc, "name": name})
        curr_person = Person()
        curr_person.name = name
        curr_person.x = int((xc - self.center[0]) * MAX_DEGREE / self.center[0])
        curr_person.y = int((self.center[1] - yc) * MAX_DEGREE / self.center[1])
        self.face_list.list.append(curr_person)

    def assign_name(
        self, left: float, top: float, bottom: float, right: float, xc: float, yc: float
    ) -> None:
        """Assign name to largest face detected"""

        crop = self.frame[top:bottom, left:right]

        img_name = f"{self.new_name}.png"
        save_path = f"{KNOWN_FACES_PATH}/{img_name}"
        cv2.imwrite(save_path, crop)
        self.process_img(img_name)

        # Update prev recognitions for tracker
        for i, face in enumerate(self.curr_faces):
            if face["x"] == xc and face["y"] == yc:
                index = i

            # cv2.circle(self.annotated_frame, (int(face["x"]), int(face["y"])), 5, (0, 255, 0), -1)

        self.curr_faces[index]["name"] = self.new_name
        self.face_list.list[index].name = self.new_name
        self.success(f"{self.new_name} face saved")

        self.new_name = ""

    def publish_follow_face(self, xc: float, yc: float, largest_face_name: str) -> None:
        """Publish coordinates for arm to follow face"""
        # Coordinates to follow largest face
        difx = 0 if xc == 0 else xc - self.center[0]
        dify = 0 if yc == 0 else self.center[1] - yc

        move_x = difx * MAX_DEGREE / self.center[0]
        move_y = dify * MAX_DEGREE / self.center[1]

        target = Point()

        # if len(self.depth_image) > 0:
        #     point2D = (xc, yc)
        #     depth = get_depth(self.depth_image, point2D)
        #     point3D = deproject_pixel_to_point(self.imageInfo, point2D, depth)
        #     point3D = float(point3D[0]), float(point3D[1]), float(point3D[2])
        #     target.z = point3D[2]

        target.x = move_x
        target.y = move_y

        self.follow_publisher.publish(target)

        person_seen = String()
        person_seen.data = largest_face_name

        self.name_publisher.publish(person_seen)
        self.person_list_publisher.publish(self.face_list)

    def run(self) -> None:
        """Run face recognition algorithm"""

        if self.image is None:
            self.get_logger().info("No image")
            return
        self.annotated_frame = self.image

        if self.id == self.processing_id:
            # self.get_logger().info("Skipping image")
            return

        self.processing_id = self.id

        self.frame = self.image
        self.annotated_frame = self.frame.copy()
        self.center = [self.frame.shape[1] / 2, self.frame.shape[0] / 2]

        resized_frame = cv2.resize(
            self.frame, (0, 0), fx=RESIZE_FACTOR, fy=RESIZE_FACTOR
        )

        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(resized_frame)
        # print("running")
        # return

        largest_area = 0
        follow_face_params = None
        largest_area_params = None
        largest_face_name = ""

        self.curr_faces = []
        self.face_list = PersonList()
        detected = False
        face_encodings = None

        # Process each face
        for i, location in enumerate(face_locations):
            # Center of current face
            scale_factor = 1 / RESIZE_FACTOR
            centerx = (location[3] + (location[1] - location[3]) / 2) * scale_factor
            centery = (location[0] + (location[2] - location[0]) / 2) * scale_factor

            top, right, bottom, left = [int(i * scale_factor) for i in location]
            name = "Unknown"

            # Extend bbox
            left = max(left - TRACK_THRESHOLD, 0)
            right = min(right + TRACK_THRESHOLD, self.frame.shape[1])
            top = max(0, top - TRACK_THRESHOLD)
            bottom = min(bottom + TRACK_THRESHOLD, self.frame.shape[0])

            # Tracking
            flag = False
            for prev_face in self.prev_faces:
                # If the face is within the tracking threshold
                if (abs(prev_face["x"] - centerx) < TRACK_THRESHOLD) and (
                    abs(prev_face["y"] - centery) < TRACK_THRESHOLD
                ):
                    name = prev_face["name"]
                    flag = True
                    break

            # If not a tracked face, then it needs to be processed (compare to known faces)
            if not flag:
                if face_encodings is None:
                    face_encodings = face_recognition.face_encodings(
                        resized_frame, face_locations
                    )

                face_encoding = face_encodings[i]

                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(
                    face_encoding, self.people_encodings, MATCH_THRESHOLD
                )
                face_distances = face_recognition.face_distance(
                    self.people_encodings, face_encoding
                )
                best_match_index = np.argmin(face_distances)

                # If it is known, then the name is updated
                if matches[best_match_index]:
                    name = self.people_names[best_match_index]

            xc = left + (right - left) / 2
            yc = top + (bottom - top) / 2
            area = (right - left) * (bottom - top)

            # Add face to list
            self.save_face(name, xc, yc)
            detected = True

            # Show results
            if flag:
                cv2.rectangle(
                    self.annotated_frame,
                    (left, bottom - 35),
                    (right, bottom),
                    (255, 0, 0),
                    cv2.FILLED,
                )
                cv2.rectangle(
                    self.annotated_frame, (left, top), (right, bottom), (255, 0, 0), 2
                )
            else:
                cv2.rectangle(
                    self.annotated_frame,
                    (left, bottom - 35),
                    (right, bottom),
                    (0, 0, 255),
                    cv2.FILLED,
                )
                cv2.rectangle(
                    self.annotated_frame, (left, top), (right, bottom), (0, 0, 255), 2
                )

            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(
                self.annotated_frame,
                name,
                (left + 6, bottom - 6),
                font,
                1.0,
                (255, 255, 255),
                1,
            )

            if area > largest_area:
                largest_area = area
                largest_area_params = [left, top, right, bottom]
                largest_face_name = name

            if self.follow_name == name:
                follow_face_params = [left, top, right, bottom]

        xc = 0
        yc = 0

        # For the follow face by name or area:
        if largest_area != 0:
            left, top, right, bottom = largest_area_params
            xc = left + (right - left) / 2
            yc = top + (bottom - top) / 2

            # Center of the face
            if self.new_name != "":
                largest_face_name = self.new_name
                self.assign_name(left, top, bottom, right, xc, yc)

        if self.follow_name != "area":
            if follow_face_params is not None:
                left, top, right, bottom = follow_face_params
                xc = left + (right - left) / 2
                yc = top + (bottom - top) / 2
                largest_face_name = self.follow_name
            else:
                detected = False

        self.prev_faces = self.curr_faces

        # Calculate the joint degrees for the arm to follow the face
        if detected:
            self.publish_follow_face(xc, yc, largest_face_name)
        else:
            self.name_publisher.publish(String(data=""))
        # if self.verbose:
        #    cv2.imshow("Face recognition", self.annotated_frame)
        # self.image_view = self.annotated_frame
        # self.view_pub.publish(
        #     self.bridge.cv2_to_imgmsg(self.self.annotated_frame, "bgr8")
        # )

        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     self.prev_faces = []
        self.publish_image()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = FaceRecognition()
        executor = rclpy.executors.MultiThreadedExecutor(5)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
