#include "nodes/template_node.hpp"

namespace nodes {
    IoNode::IoNode() : Node("io_node") {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                "bpc_prp_robot/buttons", 
                1, 
                std::bind(&IoNode::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "bpc_prp_robot/rgb_leds", 
            10
        );
        RCLCPP_INFO(this->get_logger(), "IO Node has been started.");
    }

    int IoNode::get_value() const {
        return value_;
    }

    void IoNode::callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got Value: %d", msg->data);
        if (msg->data == 0) publish(128);
    }

    void IoNode::publish(uint8_t led_0) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = {led_0};
        publisher_->publish(msg);
    }
}   