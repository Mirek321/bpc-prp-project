#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp> 
#include "algorithms/pid.hpp"

namespace nodes {
class MazeLoopNode : public rclcpp::Node {
public:
    explicit MazeLoopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MazeLoopNode() override = default;

private:
    // State Machine
    enum class State { CALIBRATION, CORRIDOR_FOLLOWING, ONE_WALL_FOLLOWING, INTERSECTION_STRAIGHT, DRIVE_TO_CENTER, TURNING, EXIT_CORNER };
    State current_state_ = State::CALIBRATION;

    // Subscribers / Publishers / Timer
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr side_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_ready_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr line_detected_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    algorithms::Pid pid_controller_;

    // State & Sensor Data
    bool is_imu_ready_ = false;
    bool has_new_error_ = false;
    float current_error_ = 0.0f;
    float current_front_distance_ = 999.0f;
    float current_yaw_ = 0.0f;
    float target_yaw_ = 0.0f;
    float last_turn_direction_ = 1.0f;
    float current_left_dist_ = 15.0f;
    float current_right_dist_ = 15.0f;
    float current_fl_dist_ = 15.0f;
    float current_fr_dist_ = 15.0f;
    rclcpp::Time last_callback_time_;
    rclcpp::Time detection_time_;
    rclcpp::Time hole_start_time_;
    rclcpp::Time last_state_change_time_;
    rclcpp::Time corridor_entry_time_;

    // Control Constants (BPC-PRP aligned)
    const float base_speed_ = 140.0f;
    const float max_correction_ = 80.0f;
    const float STOP_DISTANCE = 0.15f;
    const float YAW_PRECISION = 0.05f;
    const float YAW_DEADBAND = 0.03f;
    const float ERROR_DEADBAND = 0.02f;
    const float ERROR_ALPHA = 0.2f;
    const float YAW_P_GAIN = 3.5f;
    const float K_TURN_P = 30.0f;
    const float TURN_MAX_PWM = 15.0f;
    const float OPEN_DIST = 0.40f;       // >40cm = otvorený priestor
    const float MIN_LIDAR_DIST = 0.16f;  // Slepá zóna LiDARu
    const float DESIRED_HALF_WIDTH = 0.18f;
    const float EXIT_WALL_THRESHOLD = 1.8f;
    bool is_line_detected_ = false; // Premenná na uloženie stavu (true/false)
    bool exit_line_seen_ = false;
    rclcpp::Time exit_line_detect_time_;

    float target_wall_distance_ = 0.0f;
    bool following_left_wall_ = true;
    bool is_t_intersection_ = false;
    float final_correction_ = 0.0f;
    bool left_opening = false;
    bool right_opening = false;
    bool L_valid = true;
    bool R_valid = true;

    const float OPENING_THRESHOLD = 0.35f;
    const float TURN_DISTANCE = 0.25f;
    bool is_dead_end_turn_ = false;


    // --- Callbacks ---


    // Filtered error
    float filtered_error_ = 0.0f;

    // Callbacks & Helpers
    void control_loop_callback();
    void error_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void front_dist_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void side_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void imu_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void line_detected_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publish_motor_command(uint8_t left, uint8_t right);
    float normalize_angle(float angle) const;
};
} // namespace nodes