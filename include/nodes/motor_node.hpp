#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <vector> 
namespace nodes {
     class MotorNode : public rclcpp::Node {
     public:
        explicit MotorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        enum Direction{
            FORWARD = 150,
            STOP = 127,
            BACKWARD = 100
        };
         int get_value() const;
         const std::vector<uint32_t>& get_values() const;
         void set_speed(uint8_t l_speed, uint8_t r_speed);
 
     private:
        int value_ = -1;
        std::vector<uint32_t> values_ = {0,0}; 
        uint32_t first_encoder_value_l_ = 0;
        uint32_t first_encoder_value_r_ = 0;
        bool first_encoder_value_set_ = false;
        uint32_t delta_l_ = 0;
        uint32_t delta_r_ = 0;
        uint32_t rotations_l_ = 0;
        uint32_t rotations_r_ = 0;
        uint32_t last_rotations_ = 0;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_cmd_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        uint8_t motor_0_ = 127;
        uint8_t motor_1_ = 127;
        rclcpp::TimerBase::SharedPtr timer_;
        void callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
        void motor_command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg); 
        void updateEncoder(uint32_t encoder_value_l, uint32_t encoder_value_r);
        void publish_value();
     };
 }