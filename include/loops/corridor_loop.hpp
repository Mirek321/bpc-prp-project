#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "algorithms/pid.hpp"

namespace nodes {

class CorridorLoopNode : public rclcpp::Node {
public:
    explicit CorridorLoopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CorridorLoopNode() override = default;

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    algorithms::Pid pid_controller_;
    
    float current_error_ = 0.0f;
    bool has_new_error_ = false;
    rclcpp::Time last_callback_time_;
    
    float base_speed_ = 135.0f;
    float max_correction_ = 80.0f;
    float obstacle_threshold_ = 0.1f;
    
    // Debug
    int log_counter_ = 0;

    // Callbacks
    void control_loop_callback();
    void error_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void publish_motor_command(uint8_t left, uint8_t right);
};

}  // namespace nodes