#include "nodes/io_node.hpp"

using std::placeholders::_1;

namespace nodes {

IoNode::IoNode(const rclcpp::NodeOptions& options) : Node("io_node", options) {    // Listen to buttons
    button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bpc_prp_robot/buttons", 10,
        std::bind(&IoNode::on_button_callback, this, _1)
    );

    // Control LEDs
    led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "bpc_prp_robot/rgb_leds", 10
    );

    // Broadcast the Start/Stop state to other nodes
    state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "bpc_prp_robot/is_running", 10
    );

    RCLCPP_INFO(this->get_logger(), "IO Node started. Press Button 0 to toggle start/stop.");
    
    // Set initial LED state to RED (Stopped)
    set_led_color(255, 0, 0); 
}

void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    button_pressed_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Button pressed: %d", button_pressed_);

    // Let's use Button 0 as the Start/Stop toggle
    if (button_pressed_ == 0) {
        is_running_ = !is_running_; // Toggle the state

        if (is_running_) {
            RCLCPP_INFO(this->get_logger(), "ROBOT STARTED!");
            set_led_color(0, 255, 0); // Green LEDs
        } else {
            RCLCPP_INFO(this->get_logger(), "ROBOT STOPPED!");
            set_led_color(255, 0, 0); // Red LEDs
        }

        // Publish the new state so MazeLoopNode can see it
        auto state_msg = std_msgs::msg::Bool();
        state_msg.data = is_running_;
        state_publisher_->publish(state_msg);
    }
}

// Helper to easily change LED colors (Assumes an array of [R, G, B])
void IoNode::set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    // If your robot has multiple LEDs, you might need to push back more values here
    // e.g., msg.data = {r, g, b, r, g, b}; for 2 LEDs.
    msg.data = {r, g, b}; 
    led_publisher_->publish(msg);
}

int IoNode::get_button_pressed() const {
    return button_pressed_;
}

}  // namespace nodes