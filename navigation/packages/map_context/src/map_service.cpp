#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "frida_interfaces/srv/map_areas.hpp"
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>

class AreasServices : public rclcpp::Node
  {
    public:
      AreasServices() : Node("areas_wrapper")
      {
        this->declare_parameter("map_name", "default_map");
        service_ = this->create_service<frida_interfaces::srv::MapAreas>(
            "areas_json",
            std::bind(&AreasServices::areas_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        RCLCPP_INFO(this->get_logger(), "Starting AreasWrapper Node ...");
      }
    private:
      void areas_callback(
          const std::shared_ptr<frida_interfaces::srv::MapAreas::Request> request,
          std::shared_ptr<frida_interfaces::srv::MapAreas::Response> response
          ){
        (void)request; // Empty Request
        try{
          std::string package_share_directory = ament_index_cpp::get_package_share_directory("map_context");
          std::string map_areas = this->get_parameter("map_name").as_string();
          // Construct the path string first
          std::string file_path = package_share_directory + "/maps/areas/areas_" + map_areas + ".json";
          // Pass that single string to ifstream
          std::ifstream f(file_path);
          nlohmann::json data = nlohmann::json::parse(f);
          response->areas = data.dump(); 
          RCLCPP_INFO(this->get_logger(), "Map Areas Sended");
        }
        catch(...){
          RCLCPP_ERROR(this->get_logger(), "Error sending map areas!");
        }
      }
      rclcpp::Service<frida_interfaces::srv::MapAreas>::SharedPtr service_;
  };
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AreasServices>());
  rclcpp::shutdown();
  return 0;
}
