#include "nodes/imu_node.hpp"

namespace nodes {

    ImuNode::ImuNode(const rclcpp::NodeOptions& options) 
        : Node("imu_node", options) {
        
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "bpc_prp_robot/imu", 10, 
            std::bind(&ImuNode::imu_callback, this, std::placeholders::_1));

        reset_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "bpc_prp_robot/imu_reset", 10,
            std::bind(&ImuNode::reset_callback, this, std::placeholders::_1));

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
                imu_integrator_.setCalibration(calibration_samples_); // Pass to helper
                is_calibrated_ = true;
                
                RCLCPP_INFO(this->get_logger(), "IMU Calibrated. Starting...");
            }
            return;
        }

        auto ready_msg = std_msgs::msg::Bool();
        ready_msg.data = true;
        imu_ready_pub_->publish(ready_msg);

        imu_integrator_.update(gyro_z, dt);

        auto yaw_msg = std_msgs::msg::Float32();
        yaw_msg.data = imu_integrator_.getYaw();
        yaw_publisher_->publish(yaw_msg);
    }

    void ImuNode::reset_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        RCLCPP_WARN(this->get_logger(), "IMU Reset Triggered! Halting and recalibrating...");
        
        // Clear out the algorithm
        imu_integrator_.reset();
        
        // Clear out the node's tracking variables
        is_calibrated_ = false;
        calibration_samples_.clear();
        last_update_time_ = this->now();

        // Broadcast that the IMU is no longer ready
        auto ready_msg = std_msgs::msg::Bool();
        ready_msg.data = false;
        imu_ready_pub_->publish(ready_msg);
    }

} // namespace nodes