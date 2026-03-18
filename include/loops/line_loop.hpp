#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "algorithms/pid.hpp"

namespace nodes {
     class LineLoopNode : public rclcpp::Node {
     public:
      explicit LineLoopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
 
     private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
        rclcpp::TimerBase::SharedPtr control_timer_;
        
        algorithms::Pid pid_controller_; 
        float base_speed_ = 140.0f;
        float last_error_ = 0.0f;
        int log_counter_ = 0;

        void control_loop_callback(); 
        void line_loop_timer_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void error_callback(const std_msgs::msg::Float32::SharedPtr msg); 
        void publish_motor_command(uint8_t left_speed, uint8_t right_speed);

     };
 }