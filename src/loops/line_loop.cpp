#include "loops/line_loop.hpp"

namespace nodes {
    LineLoopNode::LineLoopNode(const rclcpp::NodeOptions& options) : Node("line_loop_node",options), pid_controller_(9.0, 0.0, 0.2)  {

        error_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
                "bpc_prp_robot/line_error", 
                10, 
                std::bind(&LineLoopNode::error_callback, this, std::placeholders::_1));

        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/motor_commands", 
            10);

        last_callback_time_ = this->now();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&LineLoopNode::control_loop_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Line Loop Node started @ 50Hz on own thread");
    }

    void LineLoopNode::error_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        last_error_ = msg->data;
    }
    void LineLoopNode::control_loop_callback() {
        rclcpp::Time now = this->now();
        double dt = (now - last_callback_time_).seconds();
        if (dt <= 0.0) {
            dt = 0.02;
        }
        last_callback_time_ = now;
        float error = last_error_;
        float correction = pid_controller_.step(error,dt);

        float left_speed_float = base_speed_ - correction;
        float right_speed_float = base_speed_ + correction;

        left_speed_float = std::max(127.0f, std::min(255.0f, left_speed_float));
        right_speed_float = std::max(127.0f, std::min(255.0f, right_speed_float));

        uint8_t left_speed = static_cast<uint8_t>(left_speed_float);
        uint8_t right_speed = static_cast<uint8_t>(right_speed_float);
        if (++log_counter_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Speed: L=%d R=%d | Error=%.2f", 
            left_speed, right_speed, error);
        }
        publish_motor_command(left_speed,right_speed);
        // switch(pose){
        //     case 0:
        //         RCLCPP_INFO(this->get_logger(), "Line left - turning left.");
        //         publish_motor_command(130, 140); 
        //         break;
        //     case 1:
        //         RCLCPP_INFO(this->get_logger(), "Line right - turning right.");
        //         publish_motor_command(140, 130); 
        //         break;
        //     case 2:
        //         RCLCPP_INFO(this->get_logger(), "Line in centre - going.");
        //         publish_motor_command(140, 140);
        //         break;
        //     case 3:
        //         RCLCPP_INFO(this->get_logger(), "Line lost! Stop.");
        //         publish_motor_command(140, 140);    
        //         break;
        //     default:
        //         RCLCPP_WARN(this->get_logger(), "Unknown pose: %d", pose);
        //         break;
        // }
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
    }
}   