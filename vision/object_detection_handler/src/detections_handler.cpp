#include "rclcpp/rclcpp.hpp"
#include "frida_interfaces/srv/detection_handler.hpp"
#include "firda_interfaces/msg/object_detection_array.hpp"
#include "firda_interfaces/msg/object_detection.hpp"

#include <unordered_map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


RECORDED_FRAMES

class DetectionsHandlerNode : public rclcpp::Node
{
  private:
    rclcpp::Service<frida_interfaces::srv::DetectionHandler>::SharedPtr service_;
    rclcpp::Subscription<frida_interfaces::msg::ObjectDetectionArray>::SharedPtr oda_subscriber_;
    frida_interfaces::msg::ObjectDetectionArray::SharedPtr object_detection_array_ = nullptr;
    unordered_multimap<string, frida_interfaces::msg::ObjectDetectionArray::SharedPtr> umm; 
  public:
    DetectionsHandlerNode() : Node("detections_handler_node"){
      oda_subcriber_ = this->create_subscription<frida_interfaces::msg::ObjectDetectionArray>(
        "object_detection_array", 10, std::bind(&DetectionsHandlerNode::oda_callback, this, std::placeholders::_1));
      service_ = this->create_service<frida_interfaces::srv::DetectionHandler>(
        "detection_handler", std::bind(&DetectionsHandlerNode::detection_handler_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void oda_callback(frida_interfaces::msg::ObjectDetectionArray::SharedPtr msg){
      RCLCPP_INFO(this->get_logger(), "Received ObjectDetectionArray");
      object_detection_array_ = msg->detections;
    }

    void detection_handler_callback( const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Request> request,
                                     const std::shared_ptr<frida_interfaces::srv::DetectionHandler::Response> response){
        RCLCPP_INFO(this->get_logger(), "Detection handler request received");
        
        if (object_detection_array_ == nullptr){
          RCLCPP_INFO(this->get_logger(), "No object detection array received yet");
          response->success = false;
          return;
        }

        for (auto detection : object_detection_array_){
          if (detection.label == request->label){
            response->success = true;
            response->detection = detection;
            return;
          }
        }
        response->success = false;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}