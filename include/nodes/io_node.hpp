#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

namespace nodes {

class IoNode : public rclcpp::Node {
public:
    explicit IoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    int get_button_pressed() const;

private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_publisher_;

    int button_pressed_ = -1;
    bool is_running_ = false; // Tracks if the robot should be driving

    void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void set_led_color(uint8_t r, uint8_t g, uint8_t b);
};

}  // namespace nodes