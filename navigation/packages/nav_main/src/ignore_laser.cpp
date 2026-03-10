#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <limits>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class IgnoreLaser : public rclcpp::Node
{
public:
    IgnoreLaser()
    : Node("ignore_laser")
    {
        std::string ignore_list;
        this->declare_parameter("ignore_array", "");
        this->declare_parameter("min_range", 0.12);
        this->get_parameter("ignore_array", ignore_list);
        this->get_parameter("min_range", min_range_);
        RCLCPP_INFO(this->get_logger(), "ignore_list: %s", ignore_list.c_str());
        RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range_);
        ignore_array_ = split(ignore_list, ',');

        if (ignore_array_.size() % 2 != 0) {
            RCLCPP_ERROR(this->get_logger(), "ignore_array must have an even number of elements");
        }

        for (const auto& degree : ignore_array_) {
            if (degree < -180 || degree > 180) {
                RCLCPP_ERROR(this->get_logger(), "ignore_array values should be between -180 and 180");
            }
        }

        auto qos = rclcpp::QoS(10).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_input", qos, std::bind(&IgnoreLaser::scanCallback, this, std::placeholders::_1));

        scan_fixed_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        auto new_scan = *scan;
        int count = static_cast<int>(ceil((scan->angle_max - scan->angle_min) / scan->angle_increment));

        for (int i = 0; i < count; ++i) {
            // Filter readings below minimum range
            if (new_scan.ranges[i] < min_range_) {
                new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }

            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            for (size_t j = 0; j < ignore_array_.size(); j += 2) {
                if (ignore_array_[j] < degree && degree <= ignore_array_[j + 1]) {
                    new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                    break;
                }
            }
        }
        scan_fixed_->publish(new_scan);
    }

    std::vector<double> split(const std::string &s, char delim) const
    {
        std::vector<double> elems;
        std::stringstream ss(s);
        std::string number;
        while (std::getline(ss, number, delim)) {
            elems.push_back(std::atof(number.c_str()));
        }
        return elems;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_fixed_;
    std::vector<double> ignore_array_;
    double min_range_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IgnoreLaser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
