#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
     class LineLoopNode : public rclcpp::Node {
     public:
        LineLoopNode();
         ~LineLoopNode() override = default;
 
     private:

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;

        void line_loop_timer_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void publish_motor_command(uint8_t left_speed, uint8_t right_speed);

     };
 }