#include "nodes/motor_node.hpp"
using namespace std::chrono_literals;
namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                "bpc_prp_robot/encoders", 
                1, 
                std::bind(&MotorNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IO Node has been started.");
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "bpc_prp_robot/set_motor_speeds", 
            10
        );
        RCLCPP_INFO(this->get_logger(), "IO Node has been started.");
        timer_ = this->create_wall_timer(
            1000ms, 
            std::bind(&MotorNode::publish_value, this)
        );
    }

    const std::vector<uint32_t>& MotorNode::get_values() const {
        return values_;
    }

    void MotorNode::callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        values_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Encoder1: %u, Encoder2: %u", values_[0], values_[1]);
        
    }
    void MotorNode::set_speed(uint8_t l_speed, uint8_t r_speed) {
        motor_0_ = l_speed;
        motor_1_ = r_speed;
        RCLCPP_INFO(this->get_logger(), "Speed set to: %d | %d", motor_0_, motor_1_);
    }

    void MotorNode::publish_value() {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = {motor_0_, motor_1_};
        publisher_->publish(msg);
    }

    // void MotorNode::publish(uint8_t led_0) {
    //     auto msg = std_msgs::msg::UInt8MultiArray();
    //     msg.data = {led_0};
    //     publisher_->publish(msg);
    // }
}   