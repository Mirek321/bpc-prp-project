#include "loops/maze_loop.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

MazeLoopNode::MazeLoopNode(const rclcpp::NodeOptions& options)
    : Node("Maze_loop_node", options),
      pid_controller_(15.0f, 0, 0) // P-only
{
    error_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar", 10,
        std::bind(&MazeLoopNode::error_callback, this, std::placeholders::_1));
    
    front_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/front_distance", 10,
        std::bind(&MazeLoopNode::front_dist_callback, this, std::placeholders::_1));

    side_dist_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "bpc_prp_robot/side_distances", 10,
        std::bind(&MazeLoopNode::side_dist_callback, this, std::placeholders::_1));
    
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/yaw", 10,
        std::bind(&MazeLoopNode::yaw_callback, this, std::placeholders::_1));
    
    imu_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/imu_ready", 10,
        std::bind(&MazeLoopNode::imu_ready_callback, this, std::placeholders::_1));

    line_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/line_detected", 
        10,
        std::bind(&MazeLoopNode::line_detected_callback, this, std::placeholders::_1));
        
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "bpc_prp_robot/motor_commands", 10);

    last_callback_time_ = this->now();
    hole_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    current_state_ = State::CALIBRATION;
    
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50 Hz
        std::bind(&MazeLoopNode::control_loop_callback, this));
    RCLCPP_INFO(this->get_logger(), "MazeLoopNode started @ 50Hz");
}

void MazeLoopNode::front_dist_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_front_distance_ = msg->data; }
void MazeLoopNode::side_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) { current_left_dist_ = msg->data[0]; current_right_dist_ = msg->data[1];         current_fl_dist_ = msg->data[2];
        current_fr_dist_ = msg->data[3]; }
}
void MazeLoopNode::yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_yaw_ = msg->data; }
void MazeLoopNode::error_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_error_ = msg->data; has_new_error_ = true; }
void MazeLoopNode::imu_ready_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!is_imu_ready_ && msg->data) RCLCPP_INFO(this->get_logger(), "IMU Calibration finished - starting movement!");
    is_imu_ready_ = msg->data;
}
void MazeLoopNode::line_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_line_detected_ = msg->data;
}

float MazeLoopNode::normalize_angle(float angle) const {
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void MazeLoopNode::publish_motor_command(uint8_t left, uint8_t right) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.push_back(left); msg.data.push_back(right);
    motor_pub_->publish(msg);
}

void MazeLoopNode::control_loop_callback() {
    rclcpp::Time now = this->now();
    double dt = (now - last_callback_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) dt = 0.02;
    last_callback_time_ = now;

    // --- SMART DEBUG COUNTER (Global for the function) ---
    static int debug_counter = 0;
    bool should_log = (++debug_counter % 50 == 0); // Log every ~1 second

    // Emergency stop
    if (current_state_ != State::CALIBRATION && current_front_distance_ < STOP_DISTANCE) {
        publish_motor_command(127, 127);
        if (should_log) RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP!");
        return;
    }

    switch (current_state_) {
        case State::CALIBRATION:
        {
            if (is_imu_ready_) {
                target_yaw_ = current_yaw_;
                current_state_ = State::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "IMU Ready. Starting Maze following.");
            }
        }
        break;

        case State::CORRIDOR_FOLLOWING:
        {
            const float MIN_LIDAR   = 0.16f;
            const float MAX_FOLLOW  = 1.0f;
            const float OPEN_DIST   = 0.40f; // Tu môžeš skúsiť 0.50f ak deteguje príliš skoro
            const float DESIRED     = 0.22f;

            bool L_valid = (current_left_dist_ > MIN_LIDAR && current_left_dist_ < MAX_FOLLOW);
            bool R_valid = (current_right_dist_ > MIN_LIDAR && current_right_dist_ < MAX_FOLLOW);

            bool L_open = (current_left_dist_ > OPEN_DIST);
            bool R_open = (current_right_dist_ > OPEN_DIST);
            bool front_close = (current_front_distance_ < 0.35f); // Predok blízko

            float raw_err = 0.0f;
            if (L_valid && R_valid) {
                raw_err = (current_right_dist_ - current_left_dist_) ;
            }
            else if (L_valid && !R_valid) {
                raw_err = DESIRED_HALF_WIDTH - current_left_dist_; 
            } 
            else if (!L_valid && R_valid) {
                raw_err = current_right_dist_ - DESIRED_HALF_WIDTH;
            } 

            filtered_error_ = ERROR_ALPHA * raw_err + (1.0f - ERROR_ALPHA) * filtered_error_;
            // Deadband je vypnutý podľa tvojho predchádzajúceho nastavenia, ak chceš zapnúť:
            // if (std::abs(filtered_error_) < ERROR_DEADBAND) filtered_error_ = 0.0f;

            float yaw_dev = normalize_angle(target_yaw_ - current_yaw_);
            // if (std::abs(yaw_dev) < YAW_DEADBAND) yaw_dev = 0.0f;
//             if (std::abs(yaw_dev) > 0.2f) {
//     target_yaw_ = current_yaw_;  // Reset ak je drift veľký
//     yaw_dev = 0.0f;
// }
            float lidar_cor = pid_controller_.step(filtered_error_, static_cast<float>(dt));
            float imu_cor   = yaw_dev * YAW_P_GAIN;
            float total_cor = std::clamp(lidar_cor + imu_cor, -max_correction_, max_correction_);

            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ + total_cor, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ - total_cor, 0.0f, 255.0f))
            );

            // Detekcia križovatky
            bool hole = front_close && (L_open || R_open);
            
            if (hole) {
                if ((now - hole_start_time_).seconds() > 0.3) { // Skrátená hysteréza pre rýchlejšiu odozvu
                    // current_state_ = State::DRIVE_TO_CENTER;
                    detection_time_ = now;
                    last_turn_direction_ = L_open ? -1.0f : 1.0f;
                    target_yaw_ = current_yaw_;
                    pid_controller_.reset();
                    // RCLCPP_INFO(this->get_logger(), "Intersection! Turn %s", L_open ? "LEFT" : "RIGHT");
                }
            } else {
                hole_start_time_ = now;
            }

            if (should_log) {
                RCLCPP_INFO(this->get_logger(), 
                    "[FOLLOW] L:%.2f(%s) R:%.2f(%s)| FL:%.2f | FR:%.2f | F:%.2f | Err:%.3f | Yaw:%.3f",
                    current_left_dist_, L_valid ? "V" : "X",
                    current_right_dist_, R_valid ? "V" : "X",
                    current_fl_dist_, 
                    current_fr_dist_,
                    current_front_distance_, filtered_error_, yaw_dev);
            }
        }
        break;

        // case State::DRIVE_TO_CENTER:
        // {
        //     float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
        //     float corr = std::clamp(yaw_err * 30.0f, -40.0f, 40.0f);
        //     publish_motor_command(
        //         static_cast<uint8_t>(std::clamp(base_speed_ + corr, 0.0f, 255.0f)),
        //         static_cast<uint8_t>(std::clamp(base_speed_ - corr, 0.0f, 255.0f))
        //     );

        //     if ((now - detection_time_).seconds() > 0.3) {
        //         // Používame M_PI / 3.0f (60 stupňov) ako si chcel
        //         target_yaw_ = normalize_angle(current_yaw_ + (last_turn_direction_ * (M_PI / 2.0f)));
        //         current_state_ = State::TURNING;
        //         RCLCPP_INFO(this->get_logger(), "Target yaw: %.3f rad", target_yaw_);
        //     }
        // }
        // break;

        // case State::TURNING:
        // {
        //     float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
        //     float corr = std::clamp(yaw_err * K_TURN_P, -TURN_MAX_PWM, TURN_MAX_PWM);
        //     publish_motor_command(
        //         static_cast<uint8_t>(std::clamp(127.0f - corr, 0.0f, 255.0f)),
        //         static_cast<uint8_t>(std::clamp(127.0f + corr ,0.0f, 255.0f))
        //     );

        //     if (std::abs(yaw_err) < YAW_PRECISION) {
        //         publish_motor_command(127, 127);
        //         current_state_ = State::EXIT_CORNER;
        //         detection_time_ = now; // Reset time for exit logic
        //         target_yaw_ = current_yaw_;
        //         RCLCPP_INFO(this->get_logger(), "Turn complete. Exiting.");
        //     }
            
        //     if (should_log) {
        //          RCLCPP_INFO(this->get_logger(), "[TURN] Cur: %.3f | Tgt: %.3f | Err: %.3f", 
        //                      current_yaw_, target_yaw_, yaw_err);
        //     }
        // }
        // break;

        //      case State::EXIT_CORNER:
        // {
        //     // Reset flagu pri prvom vstupe (alebo ho resetuj v TURNING)
        //     // Lepšie: Resetuj ho v DRIVE_TO_CENTER alebo na začiatku EXIT ak je time 0
            
        //     float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
        //     float imu_cor = yaw_err * 2.5f; 
        //     float total_cor = std::clamp(imu_cor, -40.0f, 40.0f);

        //     publish_motor_command(
        //         static_cast<uint8_t>(std::clamp(base_speed_ + total_cor, 0.0f, 255.0f)),
        //         static_cast<uint8_t>(std::clamp(base_speed_ - total_cor, 0.0f, 255.0f))
        //     );

        //     if (!exit_line_seen_ && is_line_detected_) {
        //         exit_line_seen_ = true;
        //         exit_line_detect_time_ = now;
        //         RCLCPP_INFO(this->get_logger(), "Line detected in EXIT!");
        //     }

        //     if (exit_line_seen_ && (now - exit_line_detect_time_).seconds() > 1.0) {
        //         RCLCPP_INFO(this->get_logger(), "Stable after line. Resuming Following.");
        //         pid_controller_.reset();
        //         target_yaw_ = current_yaw_; 
        //         current_state_ = State::CORRIDOR_FOLLOWING;
        //         exit_line_seen_ = false; // Reset pre budúcu križovatku
        //     }

        //     // Timeout ak čiara nie je
        //     if (!exit_line_seen_ && (now - detection_time_).seconds() > 3.0) {
        //          RCLCPP_WARN(this->get_logger(), "Exit timeout. Resuming.");
        //          pid_controller_.reset();
        //          target_yaw_ = current_yaw_; 
        //          current_state_ = State::CORRIDOR_FOLLOWING;
        //     }
        // }
        // break;

        default: break;
    }
    
    // --- GLOBAL STATE LOG ---
    if (should_log) {
        std::string state_str = "UNKNOWN";
        switch (current_state_) {
            case State::CALIBRATION:       state_str = "CALIBRATION"; break;
            case State::CORRIDOR_FOLLOWING: state_str = "FOLLOWING"; break;
            case State::DRIVE_TO_CENTER:   state_str = "CENTER"; break;
            case State::TURNING:           state_str = "TURNING"; break;
            case State::EXIT_CORNER:       state_str = "EXIT"; break;
        }
        RCLCPP_INFO(this->get_logger(), "CURRENT STATE: %s", state_str.c_str());
    }

} // End of control_loop_callback

} // namespace nodes