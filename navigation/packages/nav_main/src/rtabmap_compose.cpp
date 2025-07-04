#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#include <rclcpp/parameter.hpp>
// RTAB-Map composable includes
#include "rtabmap_slam/CoreWrapper.h"
#include "rtabmap_sync/rgbd_sync.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Options for the composable nodes (can include parameters, use_intra_process_comms, etc.)

  // You can load parameters here if needed
  // options.parameter_overrides({ ... });
  
  std::vector<rclcpp::Parameter> rgbd_sync_params = {
  {"approx_sync", false},
  {"use_sim_time", false},
  };

  std::vector<rclcpp::Parameter> rtabmap_params = {
  {"subscribe_rgb", false},
  {"subscribe_depth", false},
  {"use_sim_time", false},
  {"subscribe_rgbd", true},
  {"subscribe_scan", true},
  {"odom_sensor_sync", true},
  {"frame_id", "base_link"},
  {"odom_frame_id", "odom"},
  {"map_frame_id", "map"},
  {"database_path", "/workspace/rtabmap2.db"},
  {"approx_sync", true},

  // RTAB-Map parameters
  {"Rtabmap/DetectionRate", "15.0"},
  {"Mem/NotLinkedNodesKept", "false"},
  {"Kp/NNStrategy", "4"},
  {"Kp/MaxFeatures", "500"},
  {"Kp/DetectorStrategy", "8"},
  {"GFTT/Gpu", "true"},
  {"ORB/Gpu", "true"},
  {"RGBD/LinearUpdate", "0.1"},
  {"RGBD/AngularUpdate", "0.1"},
  {"RGBD/ForceOdom3DoF", "true"},
  {"RGBD/StartAtOrigin", "false"},
//   {"RGBD/ProximityPathMaxNeighbors", "0"},
  {"Optimizer/Strategy", "1"},
  {"Optimizer/Iterations", "20"},
  {"Optimizer/Epsilon", "0.0"},
  {"Optimizer/Robust", "true"},
  {"Optimizer/GravitySigma", "0.3"},
  {"g2o/Solver", "3"},
  {"Odom/Strategy", "1"},
  {"Odom/ResetCountdown", "0"},
  {"Odom/Holonomic", "false"},
  {"Odom/FilteringStrategy", "1"},
  {"Odom/ParticleSize", "400"},
  {"Reg/Strategy", "2"},
  {"Reg/Force3DoF", "true"},
  {"Vis/PnPRefineIterations", "0"},
  {"Vis/MinInliers", "20"},
  {"Vis/Iterations", "300"},
  {"Vis/FeatureType", "8"},
  {"Vis/MaxFeatures", "1000"},
  {"Vis/BundleAdjustment", "1"},
  {"GMS/WithRotation", "true"},
  {"GMS/WithScale", "true"},
  {"GMS/ThresholdFactor", "6.0"},
  {"Icp/Strategy", "1"},
  {"Icp/MaxCorrespondenceDistance", "0.1"},
  {"Icp/PointToPlane", "true"},
  {"Grid/Sensor", "2"},
  {"Grid/RangeMax", "5.0"},
  {"Grid/CellSize", "0.05"},
  {"Grid/3D", "false"},
  {"Grid/RayTracing", "false"},
  {"Grid/MaxGroundHeight", "0.1"},
  {"Grid/MaxObstacleHeight", "2.0"}
};

  rclcpp::NodeOptions rtabmap_slam_options;
  rclcpp::NodeOptions rgbd_sync_options;


  rgbd_sync_options.parameter_overrides(rgbd_sync_params);
  rgbd_sync_options
  .arguments({
    "--ros-args",
    "-r", "rgb/image:=/zed/zed_node/rgb/image_rect_color",
    "-r", "rgb/camera_info:=/zed/zed_node/rgb/camera_info", 
    "-r", "depth/image:=/zed/zed_node/depth/depth_registered"
  });


  rtabmap_slam_options.parameter_overrides(rtabmap_params);

  // Create RTAB-Map composable nodes
  auto core_wrapper_node = std::make_shared<rtabmap_slam::CoreWrapper>(rtabmap_slam_options);
  auto rgbd_sync_node = std::make_shared<rtabmap_sync::RGBDSync>(rgbd_sync_options);

  // Create single-threaded executors
  auto exec1 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto exec2 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Add nodes to their respective executors
  exec1->add_node(core_wrapper_node);
  exec2->add_node(rgbd_sync_node);

  // Launch each executor in its own thread
  std::thread thread1([&]() {
    exec1->spin();
  });

  std::thread thread2([&]() {
    exec2->spin();
  });

  // Wait for shutdown

  thread1.join();
  thread2.join();

  return 0;
}
