#include "nodes/io_node.hpp"
#include <functional>

using std::placeholders::_1;

namespace nodes {

IoNode::IoNode()
: Node("io_node")
{
    button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/buttons",
        10,
        std::bind(&IoNode::on_button_callback, this, _1)
    );
    // publisher_ = node_->create_publisher<std_msgs::msg::Float32>(topic, 1);
}

void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    button_pressed_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Button pressed: %d", button_pressed_);
}

int IoNode::get_button_pressed() const
{
    return button_pressed_;
}

}  // namespace nodes