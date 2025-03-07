#! /usr/bin/env python3
# Abstract class used for all diferent model classes

from abc import ABC, abstractmethod 
from dataclasses import dataclass
import imutils
import time
import sys
import copy
import pathlib
import numpy as np
import threading
from typing import List
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
import rclpy.time
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image, CameraInfo

class ObjectDectectorParams:
    def __init__(self, 
                 depth_active : bool = None, 
                 min_score_thresh : float = None, 
                 camera_frame : str = None, 
                 flip_image : bool = None, 
                 camera_info : CameraInfo = None):   
        self.depth_active = depth_active
        self.min_score_thresh = min_score_thresh
        self.camera_frame = camera_frame
        self.flip_image = flip_image
        self.camera_info = camera_info
    
    def __init__(self):
        self.depth_active= None
        self.min_score_thresh = None
        self.camera_frame = None
        self.flip_image = None
        self.camera_info = None

@dataclass
class BBOX():
    x : float = 0
    y : float = 0
    w : float = 0
    h : float = 0
    x1 : float = 0
    x2 : float = 0
    y1 : float = 0
    y2 : float = 0

class Detection:
    def __init__(self, label, class_id, confidence):
        self.bbox_ : BBOX = BBOX()
        self.label_ : str = label
        self.class_id_ : int = class_id
        self.confidence_ : float = confidence
        self.point_stamped_ : PointStamped       

class ObjectDectector(ABC): 
    def __init__(self, model_path : str, object_detector_params : ObjectDectectorParams):
        self.object_detector_params_ = object_detector_params
        self.model_path_ = model_path
        self.detections_ : List[Detection] = [] 

    # Abstract method
    @abstractmethod
    def _inference(self, frame):
        pass

    def inference(self,frame, depth_image = None):
        if self.object_detector_params_.flip_image:
            frame = imutils.rotate(frame, 180)
        
        visual_frame = copy.deepcopy(frame)
        self.detections_ : List[Detection] = [] 
        self._inference(visual_frame)

        # y1, x1, y2, x2
        if self.object_detector_params_.flip_image:
            for detection in self.detections_:
                detection.bbox_.y2, detection.bbox_.x2, detection.bbox_.y1, detection.bbox_.x1 = 1 - detection.bbox_.y1, 1 - detection.bbox_.x1, 1 - detection.bbox_.y2, 1 - detection.bbox_.x2 

        return (self.extract3D(depth_image), self.detections_, visual_frame)
        

    def extract3D(self, depth_image):
        object_set = {}

        if depth_image != None:
            pose_array = PoseArray()
            pose_array.header.frame_id = self.object_detector_params_.camera_frame
            pose_array.header.stamp = rclpy.time.Time()

        for detection in self.detections_:
            if not detection.class_id_ in object_set.keys() or object_set[detection.class_id_].confidence_ < detection.confidence_:
                point_3D = PointStamped(
                    header=Header(frame_id=self.object_detector_params_.camera_frame), point=Point()
                )

                if self.object_detector_params_.depth_active and len(depth_image) != 0 and depth_image != None:
                    point_2D = get2DCentroid(detection.bbox_, depth_image)
                    depth = get_depth(depth_image, point_2D)
                    point_3D_ = deproject_pixel_to_point(self.object_detector_params_.camera_info, point_2D, depth)

                    point_3D.point.x = point_3D_[0]
                    point_3D.point.y = point_3D_[1]
                    point_3D.point.z = point_3D_[2]

                    pose_array.poses.append(Pose(position=point_3D.point))
                
                    detection.point_stamped_ = point_3D
                
                object_set[detection.class_id_] = detection
        # PUBLISH POINT ARRAY
        
        #TODO: VISUALIZE MARKER
        return object_set.values()

    def getDetections(self):
        return self.detections_
    
    
0