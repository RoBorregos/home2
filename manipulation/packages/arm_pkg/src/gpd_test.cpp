#include <gpd/grasp_detector.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <gpd/util/cloud.h>
#include "std_srvs/srv/trigger.hpp" 

class grasp_detect : public rclcpp::Node
{  
    public:
    grasp_detect(): Node("grasp_detect")
    {
        grasp_detect_srv_ = this->create_service<std_srvs::srv::Trigger>("grasp_detect", std::bind(&grasp_detect::service_callback, this,std::placeholders::_1, std::placeholders::_2));
    
    }
 
  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_detect_srv_;

    void service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Getting pose");
        std::string config_filename = "/cfg/eigen_params.cfg";
        std::string pathcloud = "/tutorials/krylon.pcd";
        gpd::util::Cloud cloud(pathcloud, Eigen::Matrix3Xd::Zero(3, 1));
        gpd::GraspDetector detector(config_filename);
        detector.preprocessPointCloud(cloud);
        auto result = detector.detectGrasps(cloud);
    
        if(!result.empty()){
          result[0]->print();
        }
        
        response->success = true;


    }
   

}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasp_detect>());
  rclcpp::shutdown();
  return 0;
}