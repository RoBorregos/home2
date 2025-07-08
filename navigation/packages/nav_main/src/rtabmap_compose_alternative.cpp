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

  try {
    rclcpp::NodeOptions options;
    options.arguments({
      "--ros-args",
      "--params-file", 
      "/tmp/launch_params_cawiwumr"
    });

    auto node = std::make_shared<rtabmap_slam::CoreWrapper>(options);

    rclcpp::NodeOptions rgbd_sync_options;
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

    // Use a single MultiThreadedExecutor instead of two separate ones
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    
    executor.add_node(node);
    executor.add_node(rgbd_sync_node);

    // Spin in a separate thread
    std::thread executor_thread([&]() {
      try {
        while (rclcpp::ok() && !shutdown_requested) {
          executor.spin_once(std::chrono::milliseconds(100));
        }
      } catch (const std::exception& e) {
        // Handle exceptions during shutdown
      }
    });

    // Wait for shutdown signal
    while (rclcpp::ok() && !shutdown_requested) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Shutdown sequence
    executor.cancel();
    
    if (executor_thread.joinable()) {
      executor_thread.join();
    }

    // Clean shutdown
    executor.remove_node(node);
    executor.remove_node(rgbd_sync_node);
    
    rgbd_sync_node.reset();
    node.reset();

  } catch (const std::exception& e) {
    std::cerr << "Exception during execution: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
