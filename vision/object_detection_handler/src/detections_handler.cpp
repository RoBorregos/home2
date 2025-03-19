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


float ;

struct DetectionRecord{
  frida_interfaces::msg::ObjectDetection detection;
  uint_32_t frame_id;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct detections_handler_parameters
{
  float IOU_THRESHOLD;
  string TRAGET_FRAME;
  int RECORDED_SECONDS;
};

class DetectionsHandlerNode : public rclcpp::Node
{
  private:
    rclcpp::Service<frida_interfaces::srv::DetectionHandler>::SharedPtr service_;
    rclcpp::Subscription<frida_interfaces::msg::ObjectDetectionArray>::SharedPtr oda_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    frida_interfaces::msg::ObjectDetectionArray::SharedPtr object_detection_array_ = nullptr;
    unordered_multimap<string, DetectionRecord> umm_; 
    size_t frame_id_;
    std::chrono::time_point<std::chrono::system_clock> timestamp_;
    detections_handler_parameters params_;
  public:
    DetectionsHandlerNode() : Node("object_detection_handler_node"), frame_id_(0), timestamp_(std::chrono::system_clock::now()){
      declare_parameters();

      oda_subcriber_ = this->create_subscription<frida_interfaces::msg::ObjectDetectionArray>(
        "/detections", 10, std::bind(&DetectionsHandlerNode::oda_callback, this, std::placeholders::_1));
      service_ = this->create_service<frida_interfaces::srv::DetectionHandler>(
        "detection_handler", std::bind(&DetectionsHandlerNode::detection_handler_callback, this, std::placeholders::_1, std::placeholders::_2));
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DetectionsHandlerNode::timer_callback, this));

      get_parameter("IOU_THRESHOLD", IOU_THRESHOLD);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void declare_parameters(){
      this->declare_parameter("TRAGET_FRAME", "BASE_LINK");
      this->declare_parameter("IOU_THRESHOLD", 0.75);
      this->declare_parameter("RECORDED_SECONDS", 5);
    }

    void get_parameters(){
      params_.TRAGET_FRAME = this->get_parameter("TRAGET_FRAME").as_string();
      params_.IOU_THRESHOLD = this->get_parameter("IOU_THRESHOLD").as_float();
      params_.RECORDED_SECONDS = this->get_parameter("RECORDED_SECONDS").as_int();
    }

    void getIoU(frida_interfaces::msg::ObjectDetection detection1, frida_interfaces::msg::ObjectDetection detection2){
      float xA = max(detection1.x, detection2.x);
      float yA = max(detection1.y, detection2.y);
      float xB = min(detection1.x + detection1.width, detection2.x + detection2.width);
      float yB = min(detection1.y + detection1.height, detection2.y + detection2.height);

      float interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1);

      float box1Area = detection1.width * detection1.height;
      float box2Area = detection2.width * detection2.height;

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
      RCLCPP_INFO(this->get_logger(), "Received ObjectDetectionArray %f", frame_id_);
      object_detection_array_ = msg->detections;

      for (auto detection : object_detection_array_){
        DetectionRecord dr;
        dr.detection = detection;
        dr.frame_id = frame_id_;
        dr.timestamp = std::chrono::system_clock::now();
        if (umm_.find(detection.label) != umm_.end()){
          for (auto it = umm_.equal_range(detection.label).first; it != umm_.equal_range(detection.label).second; ++it){
            if (getIoU(it->second.detection,dr.detection) > params_.IOU_THRESHOLD){
              umm_.erase(it);
              break;
            }
          }
        } 
        umm_.insert({detection.label, dr});
      }

      frame_id_++;
    }

    geometry_msgs::msg::PointStamped transform_point(geometry_msgs::msg::PointStamped point, string target_frame){
      geometry_msgs::msg::TransformStamped t;
      
      try{
        t = tf_buffer_->lookupTransform(target_frame, point.header.frame_id, tf2::TimePointZero);
      } catch (tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return nullptr;
      }
      geometry_msgs::msg::PointStamped transformed_point;
      transformed_point.header.frame_id = target_frame;
      transformed_point.header.stamp = this->get_clock()->now();
      transformed_point.point.x = t.transform.translation.x + point.point.x;
      transformed_point.point.y = t.transform.translation.y + point.point.y;
      transformed_point.point.z = t.transform.translation.z + point.point.z;

      return transformed_point;
    }
    
    void detection_handler_callback( const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Request> request,
                                     const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Response> response){
        RCLCPP_INFO(this->get_logger(), "Detection handler request received");
        
        if (object_detection_array_ == nullptr || umm_.empty()){
          RCLCPP_INFO(this->get_logger(), "No object detection array received yet");
          response->success = false;
          return;
        }
        if (umm_.find(request->label) == umm_.end()){
          RCLCPP_INFO(this->get_logger(), "No detection with label %s found", request->label);
          response->success = false;
          return;
        }
        
        ObjectDetection closest_detection;
        float min_distance = std::numeric_limits<float>::max();
        for (auto it = umm_.equal_range(request->label).first; it != umm_.equal_range(request->label).second; ++it){
          geometry_msgs::msg::PointStamped transformed_point = transform_point(point, params_.TRAGET_FRAME);
          if (transformed_point == nullptr){
            RCLCPP_INFO(this->get_logger(), "Transform error");
            response->success = false;
            return;
          }
          float distance = sqrt(pow(transformed_point.point.x, 2) + pow(transformed_point.point.y, 2) + pow(transformed_point.point.z, 2));
          if (distance < min_distance){
            min_distance = distance;
            closest_detection = it->second.detection;
          }
        }

        response->detection = closest_detection;
        response->success = true;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}