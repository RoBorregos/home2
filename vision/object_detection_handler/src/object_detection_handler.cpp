#include "rclcpp/rclcpp.hpp"
#include "frida_interfaces/srv/detection_handler.hpp"
#include "frida_interfaces/msg/object_detection_array.hpp"
#include "frida_interfaces/msg/object_detection.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <unordered_map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <bits/stdc++.h>
#include <vector>

struct DetectionRecord
{
  frida_interfaces::msg::ObjectDetection detection;
  int frame_id;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct HandlerParams
{
  int RECORDED_SECONDS;
  double IOU_THRESHOLD;
  std::string TARGET_FRAME;
  std::string DETECTIONS_TOPIC;
  bool TRANSFROM_POINTS;
  bool VERBOSE;
};

class DetectionsHandlerNode : public rclcpp::Node
{
  private:
    rclcpp::Service<frida_interfaces::srv::DetectionHandler>::SharedPtr service_;
    rclcpp::Subscription<frida_interfaces::msg::ObjectDetectionArray>::SharedPtr oda_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<frida_interfaces::msg::ObjectDetection> object_detection_vector_;
    std::unordered_multimap<std::string, DetectionRecord> umm_; 
    size_t frame_id_;
    std::chrono::time_point<std::chrono::system_clock> timestamp_;
    HandlerParams params_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
  public:
    DetectionsHandlerNode() : Node("object_detection_handler_node"), frame_id_(0), timestamp_(std::chrono::system_clock::now()){
      declare_parameters();
      get_parameters();

      oda_subscriber_ = this->create_subscription<frida_interfaces::msg::ObjectDetectionArray>(
        params_.DETECTIONS_TOPIC, 10, std::bind(&DetectionsHandlerNode::oda_callback, this, std::placeholders::_1));
      service_ = this->create_service<frida_interfaces::srv::DetectionHandler>(
        "detection_handler", std::bind(&DetectionsHandlerNode::detection_handler_callback, this, std::placeholders::_1, std::placeholders::_2));
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DetectionsHandlerNode::timer_callback, this));

      
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void declare_parameters(){
      this->declare_parameter("DETECTIONS_TOPIC", "/detections");
      this->declare_parameter("TARGET_FRAME", "BASE_LINK");
      this->declare_parameter("TRANSFROM_POINTS", false);
      this->declare_parameter("VERBOSE", false);
      this->declare_parameter("IOU_THRESHOLD", 0.75);
      this->declare_parameter("RECORDED_SECONDS", 5);
    }

    void get_parameters(){
      params_.TARGET_FRAME = this->get_parameter("TARGET_FRAME").as_string();
      params_.DETECTIONS_TOPIC = this->get_parameter("DETECTIONS_TOPIC").as_string();
      params_.IOU_THRESHOLD = this->get_parameter("IOU_THRESHOLD").as_double();
      params_.RECORDED_SECONDS = this->get_parameter("RECORDED_SECONDS").as_int();
      params_.TRANSFROM_POINTS = this->get_parameter("TRANSFROM_POINTS").as_bool();
      params_.VERBOSE = this->get_parameter("VERBOSE").as_bool();
    }

    float getIoU(frida_interfaces::msg::ObjectDetection detection1, frida_interfaces::msg::ObjectDetection detection2){
      float xA = std::max(detection1.xmax, detection2.xmax);
      float yA = std::max(detection1.ymax, detection2.ymax);
      float xB = std::min(detection1.xmin, detection2.xmin);
      float yB = std::min(detection1.ymin, detection2.ymin);
      
      float interArea = std::max(float(0), xA - xB) * std::max(float(0), yA - yB);

      float box1Area = (detection1.xmax - detection1.xmin) * (detection1.ymax - detection1.ymin);
      float box2Area = (detection2.xmax - detection2.xmin) * (detection2.ymax - detection2.ymin);

      
      RCLCPP_INFO(this->get_logger(), "xA: %f, yA: %f, xB: %f, yB: %f", xA, yA, xB, yB);
      RCLCPP_INFO(this->get_logger(), "InterArea: %f", interArea);
      RCLCPP_INFO(this->get_logger(), "Box1Area: %f", box1Area);
      RCLCPP_INFO(this->get_logger(), "Box2Area: %f", box2Area);

      float iou = interArea / (box1Area + box2Area - interArea);
      return iou;
    }

    void timer_callback(){
      auto now = std::chrono::system_clock::now();
      for (auto it = umm_.begin(); it != umm_.end();){
        if (std::chrono::duration_cast<std::chrono::seconds>(now - it->second.timestamp).count() > params_.RECORDED_SECONDS){
          umm_.erase(it++);
        } else {
          ++it;
        }
      }
    }

    void oda_callback(frida_interfaces::msg::ObjectDetectionArray::SharedPtr msg){
      if (params_.VERBOSE){
        RCLCPP_INFO(this->get_logger(), "Received ObjectDetectionArray message");
      }

      object_detection_vector_ = msg->detections;

      if (object_detection_vector_.empty() && params_.VERBOSE){
        RCLCPP_INFO(this->get_logger(), "Empty object detection array received");
        return;
      }

      for (int i = 0; i < object_detection_vector_.size(); i++){
        DetectionRecord dr;
        dr.detection = object_detection_vector_[i];
        dr.frame_id = frame_id_;
        dr.timestamp = std::chrono::system_clock::now();
        if (umm_.find(object_detection_vector_[i].label_text) != umm_.end()){
          for (auto it = umm_.equal_range(object_detection_vector_[i].label_text).first; it != umm_.equal_range(object_detection_vector_[i].label_text).second; ++it){
            RCLCPP_INFO(this->get_logger(), "IoU: %f", getIoU(it->second.detection,dr.detection));
            if (getIoU(it->second.detection,dr.detection) > params_.IOU_THRESHOLD){
              RCLCPP_INFO(this->get_logger(), "SAME OBJECT DETECTED");
              umm_.erase(it);
              break;
            }
          }
        } 
        umm_.insert(std::make_pair(object_detection_vector_[i].label_text, dr));
      }

      //frame_id_++;
    }

    geometry_msgs::msg::PointStamped::SharedPtr transform_point(geometry_msgs::msg::PointStamped point, std::string target_frame){
      geometry_msgs::msg::TransformStamped t;
      
      try{
        t = tf_buffer_->lookupTransform(target_frame, point.header.frame_id, tf2::TimePointZero);
      } catch (tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return nullptr;
      }
      geometry_msgs::msg::PointStamped::SharedPtr transformed_point;
      transformed_point->header.frame_id = target_frame;
      transformed_point->header.stamp = this->get_clock()->now();
      transformed_point->point.x = t.transform.translation.x + point.point.x;
      transformed_point->point.y = t.transform.translation.y + point.point.y;
      transformed_point->point.z = t.transform.translation.z + point.point.z;

      return transformed_point;
    }
    
    void detection_handler_callback( const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Request> request,
                                     const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Response> response){
        RCLCPP_INFO(this->get_logger(), "Detection handler request received label: %s", request->label.c_str());
        
        if (object_detection_vector_.empty() || umm_.empty()){
          RCLCPP_INFO(this->get_logger(), "No object detection array received yet");
          response->success = false;
          return;
        }
        if (umm_.find(request->label) == umm_.end()){
          
          RCLCPP_INFO(this->get_logger(), "No detection with label %s found", request->label.c_str());
          response->success = false;
          return;
        }
        
        if (request->closest_object){
          frida_interfaces::msg::ObjectDetection closest_detection;
          float min_distance = std::numeric_limits<float>::max();
          for (auto it = umm_.equal_range(request->label).first; it != umm_.equal_range(request->label).second; ++it){
            geometry_msgs::msg::PointStamped::SharedPtr transformed_point = transform_point(it->second.detection.point3d, params_.TARGET_FRAME);
            if (transformed_point == nullptr){
              RCLCPP_INFO(this->get_logger(), "Transform error");
              response->success = false;
              return;
            }
            float distance = sqrt(pow(transformed_point->point.x, 2) + pow(transformed_point->point.y, 2) + pow(transformed_point->point.z, 2));
            if (distance < min_distance){
              min_distance = distance;
              closest_detection = it->second.detection;
            }
          }

          response->detection_array.detections = {closest_detection};
        } else {
          std::vector<frida_interfaces::msg::ObjectDetection> detections;
          for (auto it = umm_.equal_range(request->label).first; it != umm_.equal_range(request->label).second; ++it){
            detections.push_back(it->second.detection);
          }
          response->detection_array.detections = detections;
        }
        response->success = true;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionsHandlerNode>());
  rclcpp::shutdown();
  return 0;
}