#include "nodes/motor_node.hpp"
using namespace std::chrono_literals;
namespace nodes {
    MotorNode::MotorNode(const rclcpp::NodeOptions& options) : Node("motor_node",options) {

        subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "bpc_prp_robot/encoders", 
            10, 
            std::bind(&MotorNode::callback, this, std::placeholders::_1)
        );

         motor_cmd_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/motor_commands",  
            10,
            std::bind(&MotorNode::motor_command_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "bpc_prp_robot/set_motor_speeds", 
            10
        );

        RCLCPP_INFO(this->get_logger(), "MotorNode has been started.");
    }

    const std::vector<uint32_t>& MotorNode::get_values() const {
        return values_;
    }

    void MotorNode::updateEncoder(uint32_t encoder_value_l, uint32_t encoder_value_r) {
        if (!first_encoder_value_set_) {
            first_encoder_value_l_ = encoder_value_l;  
            first_encoder_value_r_ = encoder_value_r;
            first_encoder_value_set_ = true;
        }

        delta_l_ = encoder_value_l - first_encoder_value_l_;
        delta_r_ = first_encoder_value_r_ - encoder_value_r;

        rotations_l_ = delta_l_ / 576;
        rotations_r_ = delta_r_ / 576;
        
        uint32_t sum = (rotations_l_+rotations_r_)/2;
        // RCLCPP_INFO(this->get_logger(), "Encoder R Delta: %u Rotation R: %u SUM: %u", delta_r_, rotations_r_, sum);

    }

    void MotorNode::motor_command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            motor_0_ = msg->data[0];
            motor_1_ = msg->data[1];
            
            publish_value();

            static int counter = 0;
            if (++counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Motor cmd: L=%d R=%d", motor_0_, motor_1_);
            }
        }
    }
    void MotorNode::callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        values_ = msg->data;
        updateEncoder(values_[0], values_[1]);
        
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
}   