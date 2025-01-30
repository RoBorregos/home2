#!/usr/bin/env python3
# Published detections in 2D and 3D with a specified model.

import numpy as np
import argparse
import torch
#import tensorflow as tf
import cv2
import pathlib
import rclpy
import threading
import imutils
import time
from imutils.video import FPS
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from frida_interfaces.manipulation.msg import ObjectDetection, ObjectDetectionArray
import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *
import tf2_ros
import tf2_geometry_msgs
import imutils
import copy
from ultralytics import YOLO

SOURCES = {
    "VIDEO": str(pathlib.Path(__file__).parent) + "/../resources/test.mp4",
    "CAMERA": 0,
    "ROS_IMG": "/camaras/0",
}

ARGS= {
    "SOURCE": SOURCES["VIDEO"],
    "ROS_INPUT": False,
    "USE_ACTIVE_FLAG": True,
    "DEPTH_ACTIVE": False,
    "DEPTH_INPUT": "/camera/depth/image_raw",
    "CAMERA_INFO": "/camera/depth/camera_info",
    "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/",
    "LABELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/label_map.pbtxt",
    "MIN_SCORE_THRESH": 0.75,
    "VERBOSE": False,
    "CAMERA_FRAME": "xtion_rgb_optical_frame",
    "USE_YOLO8": False,
    "YOLO_MODEL_PATH": str(pathlib.Path(__file__).parent) + "/../models/yolov5s.pt",
    "FLIP_IMAGE": False,
}

class CamaraProcessing:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.imageInfo = CameraInfo()

        # Load Models
        print("[INFO] Loading models...")
        
        def loadTfModel():
            self.detect_fn = tf.saved_model.load(ARGS["MODELS_PATH"])

        def loadYoloModel():
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=ARGS["YOLO_MODEL_PATH"], force_reload=False)

        def yolov8_warmup(model, repetitions=1, verbose=False):
            # Warmup model
            startTime = time.time()
            # create an empty frame to warmup the model
            for i in range(repetitions):
                warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
                model.predict(source=warmupFrame, verbose=verbose)
            rclpy.logdebug(f"Model warmed up in {time.time() - startTime} seconds")

        def loadYolov8Model():
            self.model = YOLO(ARGS["YOLO_MODEL_PATH"])
            yolov8_warmup(self.model, repetitions=10, verbose=False)

        if ARGS["USE_YOLO8"]:
            print("[INFO] Loading Yolov8 Model")
            loadYolov8Model()
            print("[INFO] Yolov8 Model Loaded")
        else:
            print("[INFO] Loading Yolov5 Model")
            loadYoloModel()
            print("[INFO] Yolov5 Model Loaded")
        
        self.activeFlag = not ARGS["USE_ACTIVE_FLAG"]
        self.runThread = None
        self.subscriber = None
        self.handleSource()
        self.publisher = rclpy.Publisher('detections', ObjectDetectionArray, queue_size=5)
        self.image_publisher = rclpy.Publisher('detections_image', Image, queue_size=5)
        self.debug_image_publisher = rclpy.Publisher('debug_image', Image, queue_size=5)
        self.posePublisher = rclpy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        # to visualize the 3d points of detected objects
        self.objects_publisher_3d = rclpy.Publisher('detections_3d', MarkerArray, queue_size=5)

        # TFs
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        if ARGS["USE_ACTIVE_FLAG"]:
            rclpy.Subscriber('detectionsActive', Bool, self.activeFlagSubscriber)

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

        # Show OpenCV window.
        try:
            self.detections_frame = []
            rate = rclpy.Rate(60)
            while not rclpy.is_shutdown():
                if ARGS["VERBOSE"] and len(self.detections_frame) != 0:
                    cv2.imshow("Detections", self.detections_frame)
                    cv2.waitKey(1)

                if len(self.detections_frame) != 0:
                    self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8"))
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cv2.destroyAllWindows()
    
    # Callback for active flag
    def activeFlagSubscriber(self, msg):
        self.activeFlag = msg.data

    # Function to handle either a cv2 image or a ROS image.
    def handleSource(self):
        if ARGS["ROS_INPUT"]:
            self.subscriber = rclpy.Subscriber(ARGS["SOURCE"], Image, self.imageRosCallback)
            if ARGS["DEPTH_ACTIVE"]:
                self.subscriberDepth = rclpy.Subscriber(ARGS["DEPTH_INPUT"], Image, self.depthImageRosCallback)
                self.subscriberInfo = rclpy.Subscriber(ARGS["CAMERA_INFO"], CameraInfo, self.infoImageRosCallback)
        else:
            cThread = threading.Thread(target=self.cameraThread, daemon=True)
            cThread.start()

    # Function to handle a cv2 input.
    def cameraThread(self):
        cap = cv2.VideoCapture(ARGS["SOURCE"])
        frame = []
        rate = rclpy.Rate(30)
        try:
            while not rclpy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    continue
                if len(frame) == 0:
                    continue
                self.imageCallback(frame)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cap.release()

    # Function to handle a ROS input.
    def imageRosCallback(self, data):
        try:
            self.imageCallback(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
    
    # Function to handle ROS camera info input.
    def infoImageRosCallback(self, data):
        self.imageInfo = data
        self.subscriberInfo.unregister()

    # Function to handle Rate Neckbottle, TF object detection model frame rate (<10FPS) against camera input (>30FPS).
    # Process a frame only when the script finishes the process of the previous frame, rejecting frames to keep real-time idea.
    def imageCallback(self, img):
        self.rgb_image = img
        if not self.activeFlag:
            self.detections_frame = img
        elif self.runThread == None or not self.runThread.is_alive():
            self.runThread = threading.Thread(target=self.run, args=(img, ), daemon=True)
            self.runThread.start()

    # Handle FPS calculation.
    def callFps(self):	
        if self.fps != None:
            self.fps.stop()
            if ARGS["VERBOSE"]:
                print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
            self.fpsValue = self.fps.fps()

        self.fps = FPS().start()
        
        callFpsThread = threading.Timer(2.0, self.callFps, args=())
        callFpsThread.start()

    def yolo_run_inference_on_image(self, frame):
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(frame)
        output = {
            'detection_boxes': [],  # Normalized ymin, xmin, ymax, xmax
            'detection_classes': [], # ClassID 
            'detection_names': [], # Class Name
            'detection_scores': [] # Confidence
        }
        
        for *xyxy, conf, cls,names in results.pandas().xyxy[0].itertuples(index=False):
            # Normalized [0-1] ymin, xmin, ymax, xmax
            height = frame.shape[1]
            width = frame.shape[0]
            if conf < ARGS["MIN_SCORE_THRESH"]:
                continue
            output['detection_boxes'].append([xyxy[1]/width, xyxy[0]/height, xyxy[3]/width, xyxy[2]/height])
            output['detection_classes'].append(cls)
            output['detection_names'].append(names)
            output['detection_scores'].append(conf)
            # # ClassID
            # found = False
            # count = 1
            # for i in self.category_index.values():
            #     if i == cls:
            #         found = True
            #         break
            #     count += 1
            # if not found:
            #     self.category_index[count] = cls
            # output['detection_classes'].append(count)
            # # Confidence
            # output['detection_scores'].append(conf)
        output['detection_boxes'] = np.array(output['detection_boxes'])
        output['detection_classes'] = np.array(output['detection_classes'])
        output['detection_names'] = np.array(output['detection_names'])
        output['detection_scores'] = np.array(output['detection_scores'])
        return output
    
    def yolov8_run_inference_on_image(self, frame):
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(frame)
        output = {
            'detection_boxes': [],  # Normalized ymin, xmin, ymax, xmax
            'detection_classes': [], # ClassID 
            'detection_names': [], # Class Name
            'detection_scores': [] # Confidence
        }
        height = frame.shape[0]
        width = frame.shape[1]
        
        boxes = []
        confidences = []
        classids = []
        names = []

        for out in results:
                for box in out.boxes:
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                    x1 = x1/width
                    x2 = x2/width
                    y1 = y1/height
                    y2 = y2/height

                    class_id = box.cls[0].item()
                    prob = round(box.conf[0].item(), 2)

                    boxes.append([y1, x1, y2, x2])
                    confidences.append(float(prob))
                    classids.append(class_id)
                    names.append(self.model.names[class_id])
                    # print(f"Found {class_id} in {x1} {y1} {x2} {y2}")
                    # print("------------------------------")
                    # print()

        output['detection_boxes'] = np.array(boxes)
        output['detection_classes'] = np.array(classids)
        output['detection_names'] = np.array(names)
        output['detection_scores'] = np.array(confidences)
        
        return output

    # Function to run the detection model.
    def run_inference_on_image(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_np = np.expand_dims(frame, axis=0)
        input_tensor = tf.convert_to_tensor(frame_np, dtype=tf.uint8)

        if ARGS["VERBOSE"]:
            print('Predicting...')

        start_time = time.time()
        detections = self.detect_fn(input_tensor)
        end_time = time.time()
        elapsed_time = end_time - start_time
        
        if ARGS["VERBOSE"]:
            print('Done! Took {} seconds'.format(elapsed_time))

        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        output = {
            'detection_boxes': detections['detection_boxes'],
            'detection_classes': detections['detection_classes'],
            'detection_scores': detections['detection_scores']
        }
        return output

    # Handle the detection model input/output.
    def compute_result(self, frame):
        visual_frame = copy.deepcopy(frame)
        if ARGS["FLIP_IMAGE"]:
            visual_frame = imutils.rotate(frame, 180)
        if ARGS["USE_YOLO8"]:
            visual_detections = self.yolov8_run_inference_on_image(visual_frame)
        else:
            visual_detections = self.yolo_run_inference_on_image(visual_frame)
        detections = copy.deepcopy(visual_detections)
        detections["detection_boxes"] = np.array([[1 - box[2], 1 - box[3], 1 - box[0], 1 - box[1]] for box in visual_detections["detection_boxes"]]) if ARGS["FLIP_IMAGE"] else visual_detections["detection_boxes"]
        return self.get_objects(detections["detection_boxes"],
                                detections["detection_scores"],
                                detections["detection_classes"],
                                detections["detection_names"],
                                frame.shape[0],
                                frame.shape[1],
                                frame), visual_detections, visual_frame

    # This function creates the output array of the detected objects with its 2D & 3D coordinates.
    def get_objects(self, boxes, scores, classes, names, height, width, frame):
        objects = {}
        res = []

        pa = PoseArray()
        pa.header.frame_id = ARGS["CAMERA_FRAME"]
        pa.header.stamp = rclpy.Time.now()

        for index, value in enumerate(classes):
            if scores[index] > ARGS["MIN_SCORE_THRESH"]:
                if value in objects:
                    # in case it detects more that one of each object, grabs the one with higher score
                    if objects[value]['score'] > scores[index]:
                        continue
                
                point3d = PointStamped(header=Header(frame_id=ARGS["CAMERA_FRAME"]), point=Point())

                if ARGS["DEPTH_ACTIVE"] and len(self.depth_image) != 0:
                    # if frame is flipped, flip the point2D
                    point2D = get2DCentroid(boxes[index], self.depth_image)
                    #rclpy.loginfo("Point2D: " + str(point2D))
                    depth = get_depth(self.depth_image, point2D) ## in m
                    #rclpy.loginfo("Depth: " + str(depth))
                    #depth = depth / 1000 ## in mm
                    point3d_ = deproject_pixel_to_point(self.imageInfo, point2D, depth)
                    #rclpy.loginfo("point3d: " + str(point3d_))
                    point3d.point.x = point3d_[0]
                    point3d.point.y = point3d_[1]
                    point3d.point.z = point3d_[2]
                    pa.poses.append(Pose(position=point3d.point))

                objects[value] = {
                    "name": names[index],
                    "score": float(scores[index]),
                    "ymin": float(boxes[index][0]),
                    "xmin": float(boxes[index][1]),
                    "ymax": float(boxes[index][2]),
                    "xmax": float(boxes[index][3]),
                    "centroid_x": point2D[0],
                    "centroid_y": point2D[1],
                    "point3d": point3d
                }
        self.posePublisher.publish(pa)
        
        for label in objects:
            label_text = objects[label]["name"]
            detection = objects[label]
            res.append(ObjectDetection(
                    label = int(label),
                    label_text = str(label_text),
                    score = detection["score"],
                    ymin =  detection["ymin"],
                    xmin =  detection["xmin"],
                    ymax =  detection["ymax"],
                    xmax =  detection["xmax"],
                    point3d = detection["point3d"]
                ))
        # visualize here
        publish_marker_array = MarkerArray()
        
        for i, label in enumerate(objects):
            detection = objects[label]
            # generate markers for each object
            marker = Marker()
            marker.header.frame_id = ARGS["CAMERA_FRAME"]
            marker.header.stamp = rclpy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = detection["point3d"].point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.Duration(0.5)
            publish_marker_array.markers.append(marker)
        
        # print(f"Markers: {publish_marker_array}")
        self.objects_publisher_3d.publish(publish_marker_array)
        
            # publish 
        return res
    
    def visualize_detections(self, image, boxes, classes, names, scores, use_normalized_coordinates=True, max_boxes_to_draw=200, min_score_thresh=0.5, agnostic_mode=False):
        """Visualize detections on an input image."""
        
        # Convert image to BGR format (OpenCV uses BGR instead of RGB)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        for i in range(min(boxes.shape[0], max_boxes_to_draw)):
            if scores[i] > min_score_thresh:
                box = tuple(boxes[i].tolist())
                if agnostic_mode:
                    color = (0, 0, 0)
                else:
                    color = (0, 255, 0)
                ymin, xmin, ymax, xmax = box
                if use_normalized_coordinates:
                    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
                else:
                    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
                (left, right, top, bottom) = (int(left * image.shape[1]), int(right * image.shape[1]),
                                            int(top * image.shape[0]), int(bottom * image.shape[0]))
                cv2.rectangle(image, (left, top), (right, bottom), color, 2)
                # draw label, name and score
                cv2.putText(image, f"{classes[i]}: {names[i]}: {scores[i]}", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                # draw centroid 
                cv2.circle(image, (int((left + right) / 2), int((top + bottom) / 2)), 5, (0, 0, 255), -1)
        
        # Convert image back to RGB format
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        return image

    # Main function to run the detection model.
    def run(self, frame):
        frame_processed = frame
        # frame_processed = imutils.resize(frame, width=500)

        detected_objects, visual_detections, visual_image = self.compute_result(frame_processed)

        frame = self.visualize_detections(
            visual_image,
            visual_detections['detection_boxes'],
            visual_detections['detection_classes'],
            visual_detections["detection_names"],
            visual_detections['detection_scores'],
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=ARGS["MIN_SCORE_THRESH"],
            agnostic_mode=False)

        self.detections_frame = frame

        #print("PUBLISHED DATA")
        self.publisher.publish(ObjectDetectionArray(detections=detected_objects))
        self.fps.update()

def main():
    rclpy.init_node('Vision2D', anonymous=True)
    for key in ARGS:
        ARGS[key] = rclpy.get_param('~' + key, ARGS[key])
    CamaraProcessing()

if __name__ == '__main__':
    main()