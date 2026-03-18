#include "nodes/line_node.hpp"

namespace nodes {
    LineNode::LineNode(const rclcpp::NodeOptions& options) : Node("line_node",options) {
        last_process_time_ = this->now();
        subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "bpc_prp_robot/line_sensors", 
            10,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
            
        pose_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(
            "bpc_prp_robot/line_pose_discrete", 
            10);

        error_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "bpc_prp_robot/line_error", 
            10);
    }

    float LineNode::get_continuous_line_pose() const {
        return current_continuous; 
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() const {
        return current_discrete;
    }

 void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->now();

    uint16_t left = msg->data[0];
    uint16_t right = msg->data[1];

    // Calibration
    if(left < calibration_.left_min_value) calibration_.left_min_value = left;
    if(left > calibration_.left_max_value) calibration_.left_max_value = left;
    if(right < calibration_.right_min_value) calibration_.right_min_value = right;
    if(right > calibration_.right_max_value) calibration_.right_max_value = right;

    // Continuous estimation
    current_continuous = estimate_continuous_line_pose(left, right);
    
    // float l_norm = (left - (float)MIN_L_VALUE) / (MAX_L_VALUE - (float)MIN_L_VALUE);
    // float r_norm = (right - (float)MIN_R_VALUE) / (MAX_R_VALUE - (float)MIN_R_VALUE);
    
    // DiscreteLinePose current_discrete = estimate_discrete_line_pose(l_norm, r_norm);
    
    auto pose_msg = std_msgs::msg::Float32();
    pose_msg.data = current_continuous;
    error_publisher_->publish(pose_msg);
    last_process_time_ = now;

    static int counter = 0;
    if (++counter % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "Sensors: L=%u R=%u | Error=%.2f", left, right, current_continuous);
    }
}

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value) {
        float calibrated_left = (left_value - (float)MIN_L_VALUE) / (MAX_L_VALUE - (float)MIN_L_VALUE);
        float calibrated_right = (right_value - (float)MIN_R_VALUE) / (MAX_R_VALUE - (float)MIN_R_VALUE); 

        return calibrated_left - calibrated_right;
    }

    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm) {

        bool sensor_l = l_norm > threshold_l;  
        bool sensor_r = r_norm > threshold_r;

        if(sensor_l && sensor_r) {
            return DiscreteLinePose::LineBoth;
        }
        else if(!sensor_l && !sensor_r) {
            return DiscreteLinePose::LineNone;
        }
        else if(sensor_l && !sensor_r) {        
            return DiscreteLinePose::LineOnLeft; 
        }
        else {  
            return DiscreteLinePose::LineOnRight;
        }
    }
}