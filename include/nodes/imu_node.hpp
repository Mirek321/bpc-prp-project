#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

namespace nodes {

class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_ready_pub_;

    float yaw_ = 0.0f;
    float gyro_z_offset_ = 0.0f;
    bool is_calibrated_ = false;
    
    std::vector<float> calibration_samples_;
    const size_t MAX_CALIBRATION_SAMPLES = 100;
    rclcpp::Time last_update_time_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

} // namespace nodes