#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>

using namespace std;

class DashgoDriverCpp : public rclcpp::Node {
public:
    DashgoDriverCpp() : Node("dashgo_driver_cpp") {
        // Parameters
        port_ = this->declare_parameter("port", "/dev/ttyUSBStm32");
        baudrate_ = this->declare_parameter("baud", 115200);
        timeout_ = this->declare_parameter("timeout", 0.5);
        base_frame_ = this->declare_parameter("base_frame", "base_footprint");
        rate_ = this->declare_parameter("rate", 50.0);
        use_imu_ = this->declare_parameter("useImu", false);
        wheel_diameter_ = this->declare_parameter("wheel_diameter", 0.1518);
        wheel_track_ = this->declare_parameter("wheel_track", 0.375);
        encoder_resolution_ = this->declare_parameter("encoder_resolution", 42760);
        gear_reduction_ = this->declare_parameter("gear_reduction", 1.0);
        accel_limit_ = this->declare_parameter("accel_limit", 0.1);
        encoder_min_ = this->declare_parameter("encoder_min", 0);
        encoder_max_ = this->declare_parameter("encoder_max", 65535);
        imu_frame_id_ = this->declare_parameter("imu_frame_id", "imu_base");
        imu_offset_ = this->declare_parameter("imu_offset", 1.01);

        // Constants
        PID_RATE = 30.0;
        ticks_per_meter_ = (encoder_resolution_ * gear_reduction_) / (wheel_diameter_ * M_PI);
        max_accel_ = (accel_limit_ * ticks_per_meter_) / rate_;
        encoder_low_wrap_ = (encoder_max_ - encoder_min_) * 0.3 + encoder_min_;
        encoder_high_wrap_ = (encoder_max_ - encoder_min_) * 0.7 + encoder_min_;

        // Publishers & Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dashgo_odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        imu_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_angle", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DashgoDriverCpp::cmdVelCallback, this, std::placeholders::_1));

        odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Serial connection
        if (!connect()) {
            RCLCPP_FATAL(this->get_logger(), "FAILED to connect to Dashgo STM32 on port %s", port_.c_str());
            throw std::runtime_error("Serial connection failed");
        }

        resetEncoders();
        resetIMU();

        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0/rate_)), 
                                        std::bind(&DashgoDriverCpp::timerLoop, this));
        
        last_time_ = this->now();
        last_cmd_vel_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "DashgoDriverCpp initialized successfully");
    }

private:
    // Serial comm helpers
    bool connect() {
        try {
            io_service_ = std::make_shared<boost::asio::io_service>();
            serial_ = std::make_shared<boost::asio::serial_port>(*io_service_, port_);
            serial_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
            serial_->set_option(boost::asio::serial_port_base::character_size(8));
            serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return true;
        } catch (std::exception& e) {
            return false;
        }
    }

    uint8_t calculateChecksum(const std::vector<uint8_t>& data) {
        int cs = 0;
        for (auto b : data) cs += b;
        return static_cast<uint8_t>(cs % 255);
    }

    bool execute(const std::vector<uint8_t>& cmd, std::vector<uint8_t>& response_args, uint8_t& response_ack) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        try {
            // Write command
            boost::asio::write(*serial_, boost::asio::buffer(cmd));
            
            // Seek header [0xFF, 0xAA]
            uint8_t c;
            int state = 0;
            auto start_seek = std::chrono::steady_clock::now();
            while (state < 2) {
                if (boost::asio::read(*serial_, boost::asio::buffer(&c, 1)) != 1) return false;
                if (state == 0 && c == 0xFF) state = 1;
                else if (state == 1 && c == 0xAA) state = 2;
                else state = 0;

                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_seek).count() > 100) return false;
            }

            // Length
            uint8_t len;
            if (boost::asio::read(*serial_, boost::asio::buffer(&len, 1)) != 1) return false;

            // Package (Ack + Args)
            std::vector<uint8_t> package(len);
            if (boost::asio::read(*serial_, boost::asio::buffer(package.data(), len)) != len) return false;

            // Checksum (read but could be verified)
            uint8_t checksum;
            if (boost::asio::read(*serial_, boost::asio::buffer(&checksum, 1)) != 1) return false;

            response_ack = package[0];
            response_args.clear();
            if (len > 1) {
                response_args.assign(package.begin() + 1, package.end());
            }
            
            return true;
        } catch (std::exception& e) {
            return false;
        }
    }

    void resetEncoders() {
        std::vector<uint8_t> cmd = {0xFF, 0xAA, 0x01, 0x03, 0x04};
        std::vector<uint8_t> res; uint8_t ack;
        execute(cmd, res, ack);
    }

    void resetIMU() {
        std::vector<uint8_t> cmd = {0xFF, 0xAA, 0x01, 0x41, 0x42};
        std::vector<uint8_t> res; uint8_t ack;
        execute(cmd, res, ack);
    }

    void drive(int left, int right) {
        std::vector<uint8_t> payload = {0x05, 0x04};
        payload.push_back((left >> 0) & 0xFF);
        payload.push_back((left >> 8) & 0xFF);
        payload.push_back((right >> 0) & 0xFF);
        payload.push_back((right >> 8) & 0xFF);
        uint8_t cs = calculateChecksum(payload);
        
        std::vector<uint8_t> cmd = {0xFF, 0xAA, 0x05, 0x04};
        cmd.push_back((left >> 0) & 0xFF);
        cmd.push_back((left >> 8) & 0xFF);
        cmd.push_back((right >> 0) & 0xFF);
        cmd.push_back((right >> 8) & 0xFF);
        cmd.push_back(cs);
        
        std::vector<uint8_t> res; uint8_t ack;
        execute(cmd, res, ack);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_time_ = this->now();
        double x = msg->linear.x;
        double th = msg->angular.z;
        double left, right;

        if (x == 0) {
            if (th > 0.0 && th < 0.15) th = 0.15;
            else if (th > -0.15 && th < 0.0) th = -0.15;
            right = th * wheel_track_ * gear_reduction_ / 2.0;
            left = -right;
        } else if (th == 0) {
            left = right = x;
        } else {
            left = x - th * wheel_track_ * gear_reduction_ / 2.0;
            right = x + th * wheel_track_ * gear_reduction_ / 2.0;
        }

        v_des_left_ = static_cast<int>(left * ticks_per_meter_ / PID_RATE);
        v_des_right_ = static_cast<int>(right * ticks_per_meter_ / PID_RATE);
    }

    void timerLoop() {
        rclcpp::Time now = this->now();
        
        // Get Encoders
        std::vector<uint8_t> enc_cmd = {0xFF, 0xAA, 0x01, 0x02, 0x03};
        std::vector<uint8_t> res; uint8_t ack;
        if (execute(enc_cmd, res, ack) && ack == 0x00 && res.size() >= 4) {
            uint16_t left_enc = res[0] | (res[1] << 8);
            uint16_t right_enc = res[2] | (res[3] << 8);
            consecutive_errors_ = 0;
            updateOdometry(left_enc, right_enc, now);
        } else {
            consecutive_errors_++;
            if (consecutive_errors_ >= 10) {
                RCLCPP_FATAL(this->get_logger(), "Dashgo STM32 failed 10 times. Shutting down.");
                rclcpp::shutdown();
            }
        }

        // Get IMU
        if (use_imu_) {
            std::vector<uint8_t> imu_cmd = {0xFF, 0xAA, 0x01, 0x05, 0x06};
            if (execute(imu_cmd, res, ack) && ack == 0x00 && res.size() >= 10) {
                int16_t yaw_raw = static_cast<int16_t>(res[0] | (res[1] << 8));
                int16_t yaw_vel_raw = static_cast<int16_t>(res[2] | (res[3] << 8));
                int16_t acc_x_raw = static_cast<int16_t>(res[4] | (res[5] << 8));
                int16_t acc_y_raw = static_cast<int16_t>(res[6] | (res[7] << 8));
                int16_t acc_z_raw = static_cast<int16_t>(res[8] | (res[9] << 8));

                double yaw = yaw_raw / 100.0;
                double yaw_vel = yaw_vel_raw / 100.0;
                
                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = now;
                imu_msg.header.frame_id = imu_frame_id_;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, -1.0 * imu_offset_ * yaw * M_PI / 180.0);
                imu_msg.orientation.x = q.x();
                imu_msg.orientation.y = q.y();
                imu_msg.orientation.z = q.z();
                imu_msg.orientation.w = q.w();
                
                imu_msg.angular_velocity.z = yaw_vel * M_PI / 180.0;
                imu_msg.linear_acceleration.x = acc_x_raw / 100.0;
                imu_msg.linear_acceleration.y = acc_y_raw / 100.0;
                imu_msg.linear_acceleration.z = acc_z_raw / 100.0;
                
                imu_pub_->publish(imu_msg);
                
                auto angle_msg = std_msgs::msg::Float32();
                angle_msg.data = -1.0 * imu_offset_ * yaw * M_PI / 180.0;
                imu_angle_pub_->publish(angle_msg);
            }
        }

        // Base Controller (Drive)
        if (now - last_cmd_vel_time_ > rclcpp::Duration::from_seconds(timeout_)) {
            v_des_left_ = 0;
            v_des_right_ = 0;
        }

        // Acceleration ramp
        updateMotorVelocity(v_left_, v_des_left_);
        updateMotorVelocity(v_right_, v_des_right_);

        drive(v_left_, v_right_);
    }

    void updateMotorVelocity(int& current, int target) {
        if (current < target) {
            current += static_cast<int>(max_accel_);
            if (current > target) current = target;
        } else {
            current -= static_cast<int>(max_accel_);
            if (current < target) current = target;
        }
    }

    void updateOdometry(uint16_t left_enc, uint16_t right_enc, rclcpp::Time now) {
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (!initialized_) {
            enc_left_ = left_enc;
            enc_right_ = right_enc;
            initialized_ = true;
            return;
        }

        double dleft = calculateDistance(left_enc, enc_left_, l_wheel_mult_);
        double dright = calculateDistance(right_enc, enc_right_, r_wheel_mult_);

        enc_left_ = left_enc;
        enc_right_ = right_enc;

        double dxy_ave = (dleft + dright) / 2.0;
        double dth = (dright - dleft) / wheel_track_;
        double vxy = dxy_ave / dt;
        double vth = dth / dt;

        if (dxy_ave != 0) {
            double dx = cos(dth) * dxy_ave;
            double dy = -sin(dth) * dxy_ave;
            x_ += (cos(th_) * dx - sin(th_) * dy);
            y_ += (sin(th_) * dx + cos(th_) * dy);
        }
        th_ += dth;

        // Publish Odom
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = vxy;
        odom.twist.twist.angular.z = vth;
        
        odom_pub_->publish(odom);

        if (!use_imu_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "odom";
            t.child_frame_id = base_frame_;
            t.transform.translation.x = x_;
            t.transform.translation.y = y_;
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            odom_broadcaster_->sendTransform(t);
        }
    }

    double calculateDistance(uint16_t current, uint16_t last, int& mult) {
        if (current < encoder_low_wrap_ && last > encoder_high_wrap_) mult++;
        else if (current > encoder_high_wrap_ && last < encoder_low_wrap_) mult--;
        else mult = 0; // Reset wrap multiplier as python code does it (wait, python logic was different)
        
        // Actually, Python logic was:
        // if (left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap) : 
        //     self.l_wheel_mult = self.l_wheel_mult + 1
        // ... else: self.l_wheel_mult = 0
        // Wait, the else: self.l_wheel_mult = 0 in Python effectively resets it if no wrap happened.
        // This is weird but I'll follow it exactly.
        
        double diff = static_cast<double>(current) + mult * (encoder_max_ - encoder_min_) - last;
        return diff / ticks_per_meter_;
    }

    // Config
    string port_, base_frame_, imu_frame_id_;
    int baudrate_, encoder_min_, encoder_max_;
    double timeout_, rate_, wheel_diameter_, wheel_track_, encoder_resolution_, gear_reduction_, accel_limit_, imu_offset_;
    bool use_imu_;
    double PID_RATE, ticks_per_meter_, max_accel_, encoder_low_wrap_, encoder_high_wrap_;

    // State
    bool initialized_ = false;
    uint16_t enc_left_, enc_right_;
    int l_wheel_mult_ = 0, r_wheel_mult_ = 0;
    double x_ = 0, y_ = 0, th_ = 0;
    int v_left_ = 0, v_right_ = 0, v_des_left_ = 0, v_des_right_ = 0;
    int consecutive_errors_ = 0;
    rclcpp::Time last_time_, last_cmd_vel_time_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_angle_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial
    std::shared_ptr<boost::asio::io_service> io_service_;
    std::shared_ptr<boost::asio::io_service::work> work_;
    std::shared_ptr<boost::asio::serial_port> serial_;
    std::mutex serial_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashgoDriverCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
