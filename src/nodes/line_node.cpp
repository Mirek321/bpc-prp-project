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
        
        uint16_t left = msg->data[0];
        uint16_t right = msg->data[1];
        if(left < calibration_.left_min_value) calibration_.left_min_value = left;
        if(left > calibration_.left_max_value) calibration_.left_max_value = left;
        if(right < calibration_.right_min_value) calibration_.right_min_value = right;
        if(right > calibration_.right_max_value) calibration_.right_max_value = right;

        current_continuous =  estimate_continuous_line_pose(left, right);

        // Update the tracker
        last_process_time_ = now;
        // RCLCPP_INFO(this->get_logger(), "Received line sensor data: [%u, %u]", msg->data[0], msg->data[1]);
        
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value) {
        float calibrated_left = (left_value - (float)MIN_L_VALUE) / (MAX_L_VALUE - (float)MIN_L_VALUE);
        float calibrated_right = (right_value - (float)MIN_R_VALUE) / (MAX_R_VALUE - (float)MIN_R_VALUE); 

        RCLCPP_INFO(this->get_logger(), "Calibrated line sensor data: [%f, %f]", calibrated_left, calibrated_right);
        return calibrated_left - calibrated_right;
    }

    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm) {
        int sensor_l = l_norm > threshold_r;
        int sensor_r = r_norm > threshold_r;

        if(sensor_l && sensor_r){
            return DiscreteLinePose::LineBoth;
        }
        else if(!sensor_l && !sensor_r){
            return DiscreteLinePose::LineNone;
        }
        else if(!sensor_l && sensor_r){
            return DiscreteLinePose::LineOnLeft;
        }
        else if(sensor_l && !sensor_r){
            return DiscreteLinePose::LineOnRight;
        }
    }
}