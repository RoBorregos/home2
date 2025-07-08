#include "rtabmap_slam/CoreWrapper.h"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_sync/rgbd_sync.hpp"
#include <thread>
#include <signal.h>
#include <atomic>
#include <chrono>
std::atomic<bool> shutdown_requested{false};

void signalHandler(int signum) {
  shutdown_requested = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Install signal handler
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  rclcpp::NodeOptions options;
  rclcpp::NodeOptions rgbd_sync_options;

  options.arguments({
    "--ros-args",
    "--params-file", 
    "/tmp/launch_params_cawiwumr"
  });

  std::vector<rclcpp::Parameter> rgbd_sync_params = {
    {"approx_sync", false},
    {"use_sim_time", false},
  };

  rgbd_sync_options.parameter_overrides(rgbd_sync_params);
  rgbd_sync_options.arguments({
    "--ros-args",
    "-r", "rgb/image:=/zed/zed_node/rgb/image_rect_color",
    "-r", "rgb/camera_info:=/zed/zed_node/rgb/camera_info", 
    "-r", "depth/image:=/zed/zed_node/depth/depth_registered"
  });

  auto rgbd_sync_node = std::make_shared<rtabmap_sync::RGBDSync>(rgbd_sync_options);
  auto node = std::make_shared<rtabmap_slam::CoreWrapper>(options);

  rclcpp::executors::SingleThreadedExecutor executor1;
  rclcpp::executors::SingleThreadedExecutor executor2;

  executor1.add_node(node);
  executor2.add_node(rgbd_sync_node);

  // Start executors
  std::thread thread1([&]() {
    try {
      while (rclcpp::ok() && !shutdown_requested) {
        executor1.spin_once(std::chrono::milliseconds(100));
      }
    } catch (const std::exception& e) {
      // Handle any exceptions during shutdown
    }
  });

  std::thread thread2([&]() {
    try {
      while (rclcpp::ok() && !shutdown_requested) {
        executor2.spin_once(std::chrono::milliseconds(100));
      }
    } catch (const std::exception& e) {
      // Handle any exceptions during shutdown
    }
  });

  // Wait for shutdown signal
  while (rclcpp::ok() && !shutdown_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 1. First cancel the executors
  executor1.cancel();
  executor2.cancel();
  std::cout << "entro" << std::endl;
  // 2. Wait for threads to finish
  if (thread1.joinable()) thread1.join();
   std::cout << "entro1" << std::endl;
  if (thread2.joinable()) thread2.join();
   std::cout << "entro2" << std::endl;

  // 3. Remove nodes from executors before destroying them
  executor1.remove_node(node);
   std::cout << "entro3" << std::endl;
  executor2.remove_node(rgbd_sync_node);
  std::cout << "entro4" << std::endl;

  // 4. Reset shared_ptr to destroy nodes properly BEFORE rclcpp::shutdown()
  rgbd_sync_node.reset();
  std::cout << "entro5" << std::endl;
  node.reset();
  std::cout << "entro6" << std::endl;

  // 5. Finally shutdown rclcpp
  rclcpp::shutdown();
  std::cout << "entro7" << std::endl;

  return 0;
}
