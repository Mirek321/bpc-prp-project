#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <vector> 
namespace nodes {
     class MotorNode : public rclcpp::Node {
     public:
         MotorNode();
         ~MotorNode() override = default;

         int get_value() const;
         const std::vector<uint32_t>& get_values() const;
         void set_speed(uint8_t l_speed, uint8_t r_speed);
 
     private:
         int value_ = -1;
         std::vector<uint32_t> values_ = {0,0}; 

        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
         uint8_t motor_0_ = 127;
         uint8_t motor_1_ = 127;
         rclcpp::TimerBase::SharedPtr timer_;
        void callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

         void publish_value();
     };
 }