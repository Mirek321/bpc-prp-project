#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "algorithms/pid.hpp"

namespace nodes {

class CorridorLoopNode : public rclcpp::Node {
public:
    explicit CorridorLoopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CorridorLoopNode() override = default;

private:
    // Subscriptions & Publishers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_dist_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr side_dist_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    algorithms::Pid pid_controller_;
    
    // State Machine definition
    enum class State {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        DRIVE_TO_CENTER,
        TURNING,
        EXIT_CORNER,
        STOPPED
    };

    State current_state_ = State::CALIBRATION;

    // Control variables
    bool is_imu_ready_ = false;
    float current_error_ = 0.0f;
    bool has_new_error_ = false;
    
    rclcpp::Time last_callback_time_;
    rclcpp::Time last_state_change_time_;
    rclcpp::Time detection_time_; // NutnÃ© pro stav DRIVE_TO_CENTER

    float current_front_distance_ = 999.0f;
    float current_yaw_ = 0.0f;
    float target_yaw_ = 0.0f;
    float last_turn_direction_ = 1.0f;
    float current_left_dist_ = 1.0f;
    float current_right_dist_ = 1.0f;

    // Constants
    float base_speed_ = 135.0f;
    float max_correction_ = 80.0f;
    float obstacle_threshold_ = 0.1f;
    const float STOP_DISTANCE = 0.15f;
    const float YAW_PRECISION = 0.05f;
    const float TURN_SPEED = 25.0f;

    float filtered_error_ = 0.0f;
    const float ERROR_ALPHA = 0.2f;
    const float YAW_DEADBAND = 0.03f;
    const float ERROR_DEADBAND = 0.02f;

    const float MIN_TIME_BETWEEN_TURNS = 2.0f;  // [s]

    // Parametre pre IMU korekciu a detekciu zákrut
    const float YAW_P_GAIN = 2.5f;
    const float YAW_CORNER_THRESHOLD = 0.25f;   // ~14°

    // Deklarácia helper funkcie
    float normalize_angle(float angle) const;
    
    // Debug
    int log_counter_ = 0;

    // Callbacks
    void control_loop_callback();
    void error_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void publish_motor_command(uint8_t left, uint8_t right);
    void front_dist_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void side_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Inline callback for IMU status
    void imu_ready_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!is_imu_ready_ && msg->data) {
            RCLCPP_INFO(this->get_logger(), "IMU Calibration finished - starting movement!");
        }
        is_imu_ready_ = msg->data;
    }
};

}  // namespace nodes