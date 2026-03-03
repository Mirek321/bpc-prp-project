#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
     class IoNode : public rclcpp::Node {
     public:
         IoNode();
         ~IoNode() override = default;

         int get_value() const;
 
     private:
         int value_ = -1;

         rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
         rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

         void callback(const std_msgs::msg::UInt8::SharedPtr msg);

         void publish(uint8_t num);
     };
 }