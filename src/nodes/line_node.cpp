#include "nodes/line_node.hpp"

namespace nodes {
    LineNode::LineNode() : Node("line_node") {
        last_process_time_ = this->now();
        subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "bpc_prp_robot/line_sensors", 
            1,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
    }

    float LineNode::get_continuous_line_pose() const {
        // Implementation here
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() const {
        // Implementation here
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        rclcpp::Time now = this->now();
        if ((now - last_process_time_).seconds() < 0.5) {
            return; // Exit instantly, doing no work
        }
        
        // Update the tracker
        last_process_time_ = now;
        RCLCPP_INFO(this->get_logger(), "Received line sensor data: [%u, %u]", msg->data[0], msg->data[1]);
        
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value) {
        // Implementation here
    }

    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm) {
        // Implementation here
    }
}