#include "loops/line_loop.hpp"

namespace nodes {
    LineLoopNode::LineLoopNode() : Node("line_loop_node") {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/bpc_prp_robot/line_pose_discrete", 
                1, 
                std::bind(&LineLoopNode::line_loop_timer_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 
            10);
        
        RCLCPP_INFO(this->get_logger(), "Line Loop Node started.");
    }

    void LineLoopNode::line_loop_timer_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        uint8_t pose = msg->data;
        switch(pose){
            case 0:
                RCLCPP_INFO(this->get_logger(), "Line left - turning left.");
                publish_motor_command(130, 140); 
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Line right - turning right.");
                publish_motor_command(140, 130); 
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Line in centre - going.");
                publish_motor_command(140, 140);
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Line lost! Stop.");
                publish_motor_command(140, 140);    
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown pose: %d", pose);
                break;
        }
        // auto executor_thread = std::thread(&[executor](){executor->spin();});
        // while(rclcpp::OK()){

        //     std::thread::sleep_for(std::chrono::milliseconds(100));
        // }
        
    }
    void LineLoopNode::publish_motor_command(uint8_t left_speed, uint8_t right_speed) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.push_back(left_speed);   
        msg.data.push_back(right_speed);  
        motor_publisher_->publish(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Motor command: L=%d, R=%d", left_speed, right_speed);
    }
}   