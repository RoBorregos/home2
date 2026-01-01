#!/usr/bin/env python3
import cv2
import numpy as np
import torch
import torch.nn as nn
from ultralytics import YOLO
import tqdm

from vision_general.utils.deep_sort import nn_matching, preprocessing
from vision_general.utils.deep_sort.detection import Detection
from vision_general.utils.deep_sort.tracker import Tracker

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import SetBool, Trigger
from frida_interfaces.srv import TrackBy

from vision_general.utils.reid_model import (
    load_network,
    extract_feature_from_img,
    get_structure,
)
from vision_general.utils.calculations import (
    get2DCentroid,
    get_depth,
    deproject_pixel_to_point,
)

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
)

# Configuration constants
CONF_THRESHOLD = 0.6
NMS_MAX_OVERLAP = 0.3
MIN_DETECTION_HEIGHT = 50
MAX_COSINE_DISTANCE = 0.3S
NN_BUDGET = 100
MIN_CONFIDENCE = 0.4


class ReIDNode(Node):
    def __init__(self):
        super().__init__("reid_node")
        self.get_logger().info("Initializing ReID Node...")

        # Bridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to ZED camera topics
        self.image_subscriber = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_callback,
            10
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10
        )

        # Publishers for visualization and results
        self.tracked_image_publisher = self.create_publisher(
            Image,
            "/vision/reid/tracked_image",
            10
        )

        self.detection_image_publisher = self.create_publisher(
            Image,
            "/vision/reid/detection_image",
            10
        )

        self.tracked_positions_publisher = self.create_publisher(
            PointStamped,
            "/vision/reid/tracked_positions",
            10
        )

        # Services
        self.enable_tracking_service = self.create_service(
            SetBool,
            "/vision/reid/enable_tracking",
            self.enable_tracking_callback
        )

        self.reset_tracker_service = self.create_service(
            Trigger,
            "/vision/reid/reset_tracker",
            self.reset_tracker_callback
        )

        self.get_tracking_status_service = self.create_service(
            Trigger,
            "/vision/reid/get_status",
            self.get_tracking_status_callback
        )

        # Setup models and variables
        self.setup()

        # Create timer for processing
        self.create_timer(0.1, self.process_frame)

        self.get_logger().info("ReID Node initialized successfully!")

    def setup(self):
        """Initialize models, variables, and Deep SORT tracker"""
        self.tracking_enabled = False
        self.current_image = None
        self.current_depth = None
        self.camera_info = None
        self.frame_id = CAMERA_FRAME
        self.image_timestamp = None
        
        # Frame counter
        self.frame_idx = 0
        
        # Store tracked persons
        self.tracked_persons = {}  # track_id: {bbox, feature, position, etc}
        
        self.get_logger().info("Loading models...")
        pbar = tqdm.tqdm(total=4, desc="Loading models")

        # Load YOLO for person detection
        self.yolo_model = YOLO("yolov8n.pt")
        pbar.update(1)

        # Load ReID model
        try:
            structure = get_structure()
            pbar.update(1)
            
            self.reid_model = load_network(structure)
            self.reid_model.classifier.classifier = nn.Sequential()
            pbar.update(1)
            
            if torch.cuda.is_available():
                self.reid_model = self.reid_model.cuda()
                self.get_logger().info("Using GPU for ReID")
            else:
                self.get_logger().info("Using CPU for ReID")
            
            pbar.update(1)
        except Exception as e:
            self.get_logger().error(f"Error loading ReID model: {e}")
            self.reid_model = None

        metric = nn_matching.NearestNeighborDistanceMetric(
            "cosine",
            MAX_COSINE_DISTANCE,
            NN_BUDGET
        )
        self.tracker = Tracker(metric)

        pbar.close()
        self.get_logger().info("Models loaded successfully")

    def image_callback(self, msg):
        """Callback for receiving images from ZED camera"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def depth_callback(self, msg):
        """Callback for receiving depth images from ZED camera"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error in depth callback: {e}")

    def camera_info_callback(self, msg):
        """Callback for receiving camera information"""
        self.camera_info = msg

    def enable_tracking_callback(self, request, response):
        """Service callback to enable/disable tracking"""
        self.tracking_enabled = request.data
        response.success = True
        response.message = f"Tracking {'enabled' if self.tracking_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        
        if self.tracking_enabled:
            self.frame_idx = 0
        
        return response

    def reset_tracker_callback(self, request, response):
        """Service callback to reset the tracker"""
        if self.tracker is not None:
            metric = nn_matching.NearestNeighborDistanceMetric(
                "cosine",
                MAX_COSINE_DISTANCE,
                NN_BUDGET
            )
            self.tracker = Tracker(metric)
        
        self.tracked_persons.clear()
        self.frame_idx = 0
        
        response.success = True
        response.message = "Tracker reset successfully"
        self.get_logger().info(response.message)
        return response

    def get_tracking_status_callback(self, request, response):
        """Service callback to get tracking status"""
        response.success = self.tracking_enabled
        
        if self.tracking_enabled and self.tracker is not None:
            num_tracks = len(self.tracker.tracks)
            response.message = f"Tracking enabled. Active tracks: {num_tracks}"
        else:
            response.message = "Tracking disabled"
        
        return response

    def detect_persons(self, image):
        """Detect persons using YOLO and extract features"""
        detections = []
        
        # Run YOLO detection
        results = self.yolo_model(image, classes=[0], verbose=False)  # class 0 is person
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf < MIN_CONFIDENCE:
                    continue
                
                # Get bounding box in tlwh format (top-left-width-height)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bbox = [x1, y1, x2 - x1, y2 - y1]
                
                # Skip small detections
                if bbox[3] < MIN_DETECTION_HEIGHT:
                    continue
                
                # Extract ReID features
                feature = None
                if self.reid_model is not None:
                    try:
                        person_crop = image[y1:y2, x1:x2]
                        if person_crop.size > 0:
                            feature = extract_feature_from_img(person_crop, self.reid_model)
                            feature = feature.cpu().numpy().flatten()
                    except Exception as e:
                        self.get_logger().debug(f"Error extracting features: {e}")
                        feature = np.zeros(512)  # Fallback to zero vector
                else:
                    feature = np.zeros(512)
                
                if feature is not None:
                    detections.append(Detection(bbox, conf, feature))
        
        return detections

    def process_frame(self):
        """Main processing loop"""
        if not self.tracking_enabled or self.current_image is None:
            return
        
        try:
            image = self.current_image.copy()
            self.frame_idx += 1
            
            # Detect persons
            detections = self.detect_persons(image)
            
            if not detections:
                self.publish_visualization(image, [])
                return
            
            # Apply NMS
            if detections:
                boxes = np.array([d.tlwh for d in detections])
                scores = np.array([d.confidence for d in detections])
                indices = preprocessing.non_max_suppression(
                    boxes,
                    NMS_MAX_OVERLAP,
                    scores
                )
                detections = [detections[i] for i in indices]
            
            # Update tracker
            tracks = []
            if self.tracker is not None:
                self.tracker.predict()
                self.tracker.update(detections)
                tracks = self.tracker.tracks
            
            # Publish results and visualizations
            self.publish_visualization(image, tracks)
            self.publish_tracked_positions(tracks)
            
        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {e}")

    def publish_visualization(self, image, tracks):
        """Draw bounding boxes and publish visualization"""
        vis_image = image.copy()
        
        if tracks:
            for track in tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    continue
                
                bbox = track.to_tlbr()  # top-left-bottom-right
                track_id = track.track_id
                
                # Draw bounding box
                cv2.rectangle(
                    vis_image,
                    (int(bbox[0]), int(bbox[1])),
                    (int(bbox[2]), int(bbox[3])),
                    (0, 255, 0),
                    2
                )
                
                # Draw track ID
                cv2.putText(
                    vis_image,
                    f"ID: {track_id}",
                    (int(bbox[0]), int(bbox[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )
                
                # Store tracked person info
                self.tracked_persons[track_id] = {
                    'bbox': bbox,
                    'frame': self.frame_idx
                }
        
        # Publish visualization
        try:
            tracked_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            tracked_msg.header.stamp = self.image_timestamp if self.image_timestamp else self.get_clock().now().to_msg()
            tracked_msg.header.frame_id = self.frame_id
            self.tracked_image_publisher.publish(tracked_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {e}")

    def publish_tracked_positions(self, tracks):
        """Publish 3D positions of tracked persons"""
        if self.current_depth is None or self.camera_info is None:
            return
        
        for track in tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            
            bbox = track.to_tlwh()
            centroid_2d = get2DCentroid(bbox)
            
            # Get depth at centroid
            try:
                depth = get_depth(
                    self.current_depth,
                    int(centroid_2d[0]),
                    int(centroid_2d[1])
                )
                
                if depth > 0:
                    # Deproject to 3D
                    point_3d = deproject_pixel_to_point(
                        self.camera_info,
                        centroid_2d,
                        depth
                    )
                    
                    # Publish position
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.image_timestamp if self.image_timestamp else self.get_clock().now().to_msg()
                    point_msg.header.frame_id = self.frame_id
                    point_msg.point.x = point_3d[0]
                    point_msg.point.y = point_3d[1]
                    point_msg.point.z = point_3d[2]
                    
                    self.tracked_positions_publisher.publish(point_msg)
                    
            except Exception as e:
                self.get_logger().debug(f"Error getting 3D position: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        reid_node = ReIDNode()
        rclpy.spin(reid_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'reid_node' in locals():
            reid_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
