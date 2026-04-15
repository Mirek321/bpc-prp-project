#include "nodes/imu_node.hpp"

namespace nodes {

ImuNode::ImuNode(const rclcpp::NodeOptions& options) 
    : Node("imu_node", options) {
    
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "bpc_prp_robot/imu", 10, 
        std::bind(&ImuNode::imu_callback, this, std::placeholders::_1));

    yaw_publisher_ = this->create_publisher<std_msgs::msg::Float32>("bpc_prp_robot/yaw", 10);
    imu_ready_pub_= this->create_publisher<std_msgs::msg::Bool>("bpc_prp_robot/imu_ready", 10);

    last_update_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "ImuNode started. Waiting for calibration...");
}

void ImuNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    float gyro_z = msg->angular_velocity.z;

    if (!is_calibrated_) {
        calibration_samples_.push_back(msg->angular_velocity.z);
        
        if (calibration_samples_.size() >= MAX_CALIBRATION_SAMPLES) {
            float sum = std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0f);
            gyro_z_offset_ = sum / calibration_samples_.size();
            is_calibrated_ = true;
            
            RCLCPP_INFO(this->get_logger(), "IMU Calibrated. Starting...");
        }
        return;
    }

    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    imu_ready_pub_->publish(ready_msg);

    float adjusted_gyro_z = gyro_z - gyro_z_offset_;
    yaw_ += adjusted_gyro_z * dt;

    auto yaw_msg = std_msgs::msg::Float32();
    yaw_msg.data = yaw_;
    yaw_publisher_->publish(yaw_msg);
}

} // namespace nodes