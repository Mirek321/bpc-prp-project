#include "loops/maze_loop.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

MazeLoopNode::MazeLoopNode(const rclcpp::NodeOptions& options)
    : Node("Maze_loop_node", options),
      pid_controller_(15.0f, 0.2f, 2.0f),
      pid_controller_imu_(10.0f, 1.5f, 0.0f)
{
    running_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/is_running", 10,
        std::bind(&MazeLoopNode::running_callback, this, std::placeholders::_1));
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

    imu_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "bpc_prp_robot/imu_reset", 10);

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

void MazeLoopNode::running_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_running_enabled_ = msg->data;
}

void MazeLoopNode::trigger_imu_reset() {
    // Stop the motors immediately
    publish_motor_command(127, 127);
    
    // Publish the reset trigger to the IMU node
    auto msg = std_msgs::msg::Empty();
    imu_reset_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent IMU reset command. Halting robot.");
    
    // Reset the Maze Loop state machine
    current_state_ = State::CALIBRATION;
    is_imu_ready_ = false; // Force it to wait for the new ready signal
    
    // Reset time trackers to prevent system/ROS time crashes
    rclcpp::Time now = this->now();
    last_callback_time_ = now;
    hole_start_time_ = now;
    detection_time_ = now;
    exit_line_detect_time_ = now;
    corridor_entry_time_ = now;
    last_state_change_time_ = now;
}

void MazeLoopNode::control_loop_callback() {
    if (!is_running_enabled_) {
        publish_motor_command(127, 127); // Stop motors if not running
        return;
    }

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

    float final_correction = 0.0f;

    switch (current_state_) {
        case State::CALIBRATION:
        {
            publish_motor_command(127, 127);   
            if (is_imu_ready_) {
                target_yaw_ = current_yaw_;
                current_state_ = State::CORRIDOR_FOLLOWING;
                last_state_change_time_ = now;
                corridor_entry_time_ = now;
                RCLCPP_INFO(this->get_logger(), "IMU Ready. Starting Maze following.");
            }
        }
        break;

   case State::CORRIDOR_FOLLOWING:
        {
            // Tvoje logika detekce křižovatek a rohů
            // if ((now - corridor_entry_time_).seconds() > 1.5f) {

             if (current_front_distance_ < 0.25f && 
                    current_left_dist_ < 0.25f && 
                    current_right_dist_ < 0.25f) {
                    
                    RCLCPP_INFO(this->get_logger(), "🚫 DEAD END → 180° turn");
                    
                    target_yaw_ = normalize_angle(current_yaw_ + static_cast<float>(M_PI));  // flip direction
                    last_turn_direction_ = 1.0f;  // ľubovoľné, len pre log
                    is_dead_end_turn_ = true;     // ← FLAG: toto je dead-end turn
                    detection_time_ = now;
                    current_state_ = State::TURNING;
                    return;
                }

                L_valid = (current_left_dist_ > MIN_LIDAR_DIST);
                R_valid = (current_right_dist_ > MIN_LIDAR_DIST);

                if (!L_valid) {
                    current_state_ = State::ONE_WALL_FOLLOWING;
                    following_left_wall_ = false; // sledujeme pravou, levá zmizela
                    target_wall_distance_ = DESIRED_HALF_WIDTH;
                    return;
                } 
                else if (!R_valid) {
                    current_state_ = State::ONE_WALL_FOLLOWING;
                    following_left_wall_ = true; // sledujeme levou, pravá zmizela
                    target_wall_distance_ = DESIRED_HALF_WIDTH;
                    return;
                }
                left_opening = (current_left_dist_ > OPENING_THRESHOLD);
                right_opening = (current_right_dist_ > OPENING_THRESHOLD);

                if (left_opening && right_opening) {
                    detection_time_ = now;
                    current_state_ = State::INTERSECTION_STRAIGHT;
                    is_t_intersection_ = (current_front_distance_ < 0.5f);
                    return;
                } 
                else if (left_opening) {
                    current_state_ = State::ONE_WALL_FOLLOWING;
                    following_left_wall_ = false; // sledujeme pravou, levá zmizela
                    target_wall_distance_ = DESIRED_HALF_WIDTH;
                    return;
                } 
                else if (right_opening) {
                    current_state_ = State::ONE_WALL_FOLLOWING;
                    following_left_wall_ = true; // sledujeme levou, pravá zmizela
                    target_wall_distance_ = DESIRED_HALF_WIDTH;
                    return;
                }
            // }
   

            // Výpočet korekce: PID z chyby Lidaru + P složka z YAW pro směrovou stabilitu
            float yaw_error = normalize_angle(target_yaw_ - current_yaw_);
            final_correction = pid_controller_.step(current_error_, dt) + (yaw_error * 3.0f);

              if (should_log) {
                RCLCPP_INFO(this->get_logger(), 
                    "[FOLLOW] L:%.2f(%s) R:%.2f(%s) F:%.2f | FL:%.2f | FR:%.2f| Err:%.3f | Yaw:%.3f",
                    current_left_dist_, left_opening ? "V" : "X",
                    current_right_dist_, right_opening ? "V" : "X",
                    current_fl_dist_, current_fr_dist_,
                    current_front_distance_, final_correction, yaw_error);
            }
        }
        break;

        case State::ONE_WALL_FOLLOWING:
        {
            if (current_front_distance_ < 0.25f && 
                current_left_dist_ < 0.25f && 
                current_right_dist_ < 0.25f) {
                
                RCLCPP_INFO(this->get_logger(), "🚫 DEAD END → 180° turn");
                
                target_yaw_ = normalize_angle(current_yaw_ + static_cast<float>(M_PI));  // flip direction
                last_turn_direction_ = 1.0f;  // ľubovoľné, len pre log
                is_dead_end_turn_ = true;     // ← FLAG: toto je dead-end turn
                detection_time_ = now;
                current_state_ = State::TURNING;
                return;
            }

            constexpr float CORRIDOR_RETURN_THRESH = 0.30f;
                if (current_left_dist_ < CORRIDOR_RETURN_THRESH && current_right_dist_ < CORRIDOR_RETURN_THRESH && R_valid && L_valid) {
                RCLCPP_INFO(this->get_logger(), "🔄 Both walls detected - resuming CORRIDOR_FOLLOWING");
                corridor_entry_time_ = now;
                target_yaw_ = current_yaw_;       // Lock current heading
                pid_controller_.reset();          // Clear wall-following integral (bpc-prp-6-pid)
                current_state_ = State::CORRIDOR_FOLLOWING;
                return; // Skip rest of loop this cycle
            }
            // Kontrola, zda se z rohu nevyklubala křižovatka
            if (current_left_dist_ > OPENING_THRESHOLD && current_right_dist_ > OPENING_THRESHOLD) {
                detection_time_ = now;
                current_state_ = State::INTERSECTION_STRAIGHT;
                is_t_intersection_ = (current_front_distance_ < 0.5f);
                return;
            }
            // "Umělá" chyba podle vzdálenosti od jedné stěny
            float one_wall_error = following_left_wall_ ? 
                                   (current_left_dist_ - target_wall_distance_) : 
                                   (target_wall_distance_ - current_right_dist_);

            if (current_front_distance_ < TURN_DISTANCE) {
                // Určení směru zatáčení podle toho, která stěna chybí
                float turn_dir = following_left_wall_ ? -1.0f : 1.0f; 
                target_yaw_ = normalize_angle(target_yaw_ + (turn_dir * M_PI / 2.0f));
                last_turn_direction_ = turn_dir;
                final_correction = 0.0f; 
                publish_motor_command(127, 127); 
                detection_time_ = now;
                current_state_ = State::TURNING;
                return;
            }

            final_correction = pid_controller_.step(one_wall_error, dt) + (normalize_angle(target_yaw_ - current_yaw_) * 0.8f);

            L_valid = (current_left_dist_ > MIN_LIDAR_DIST);
            R_valid = (current_right_dist_ > MIN_LIDAR_DIST);

            if (should_log) {
                RCLCPP_INFO(this->get_logger(), 
                    "[FOLLOW] L:%.2f(%s) R:%.2f(%s) F:%.2f | FL:%.2f | FR:%.2f| Err:%.3f | ValidR:%d ValidL:%d",
                    current_left_dist_, left_opening ? "V" : "X",
                    current_right_dist_, right_opening ? "V" : "X",
                    current_fl_dist_, current_fr_dist_,
                    current_front_distance_, final_correction, R_valid, L_valid);
            }
        }
        break;

        case State::INTERSECTION_STRAIGHT:
        {
            if (should_log) {
                RCLCPP_INFO(this->get_logger(), 
                    "[FOLLOW] L:%.2f(%s) R:%.2f(%s) F:%.2f | FL:%.2f | FR:%.2f| Err:%.3f",
                    current_left_dist_, left_opening ? "V" : "X",
                    current_right_dist_, right_opening ? "V" : "X",
                    current_fl_dist_, current_fr_dist_,
                    current_front_distance_, final_correction);
            }
            if (!exit_line_seen_ && is_line_detected_) {
                exit_line_seen_ = true;
                exit_line_detect_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Line detected in INTERSECTION!");
            }

            if (exit_line_seen_ && (now - exit_line_detect_time_).seconds() > 1.0) {
                RCLCPP_INFO(this->get_logger(), "Stable after line. Resuming Following.");
                pid_controller_.reset();
                target_yaw_ = current_yaw_; 
                current_state_ = State::CORRIDOR_FOLLOWING;
                exit_line_seen_ = false; // Reset pre budúcu križovatku
                return;
            }

            float yaw_error = normalize_angle(target_yaw_ - current_yaw_);
            final_correction = yaw_error * 4.5; // Čistě IMU řízení přes křižovatku

            if (is_t_intersection_ && current_front_distance_ < TURN_DISTANCE) {
                // T-křižovatka: vyber volnou cestu
                last_turn_direction_ = (current_right_dist_ > OPENING_THRESHOLD) ? -1.0f : 1.0f;
                target_yaw_ = normalize_angle(target_yaw_ - (last_turn_direction_ * M_PI / 2.0f));
                detection_time_ = now;
                current_state_ = State::TURNING;
                return;
            } 
            else if (!is_t_intersection_ && current_left_dist_ < OPENING_THRESHOLD && current_right_dist_ < OPENING_THRESHOLD) {
                corridor_entry_time_ = now;
                current_state_ = State::CORRIDOR_FOLLOWING;
            }
            // else if((!is_t_intersection_ ) && 
            //         (now - corridor_entry_time_).seconds() > 2.0f) { // Počkaj 200ms na stabilizáciu
            // RCLCPP_INFO(this->get_logger(), "🔄 Forced right turn at intersection");
        
        // target_yaw_ = normalize_angle(target_yaw_ + static_cast<float>(M_PI) / 2.0f); // +90°
        // last_turn_direction_ = 1.0f;
        // detection_time_ = now;
        // current_state_ = State::TURNING;
        // return; // ← Dôležité!
        }
        break;

   
        case State::TURNING:
       {
            float yaw_err = normalize_angle(target_yaw_ - current_yaw_);

            float corr = std::clamp(pid_controller_imu_.step(yaw_err, dt), -TURN_MAX_PWM, TURN_MAX_PWM);            
                publish_motor_command(
                static_cast<uint8_t>(std::clamp(127.0f - corr, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(127.0f + corr, 0.0f, 255.0f))
            );


            if (std::abs(yaw_err) < YAW_PRECISION) {
                publish_motor_command(127, 127);
                current_state_ = State::EXIT_CORNER;
                detection_time_ = now; // Reset time for exit logic
                target_yaw_ = current_yaw_;
                is_dead_end_turn_ = false; // Reset flagu pro další zatáčku
                //trigger_imu_reset();
                RCLCPP_INFO(this->get_logger(), "Turn complete. Exiting.");
                return;
            }
            
            if (should_log) {
                 RCLCPP_INFO(this->get_logger(), "[TURN] Cur: %.3f | Tgt: %.3f | Err: %.3f", 
                             current_yaw_, target_yaw_, yaw_err);
            }
        }
break;
        
        case State::EXIT_CORNER:
        {
            
            // Reset flagu pri prvom vstupe (alebo ho resetuj v TURNING)
            // Lepšie: Resetuj ho v DRIVE_TO_CENTER alebo na začiatku EXIT ak je time 0
            
            float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
            float imu_cor = yaw_err * 4.0f; 
            float total_cor = std::clamp(imu_cor, -40.0f, 40.0f);

            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ - total_cor, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ + total_cor, 0.0f, 255.0f))
            );

            if (!exit_line_seen_ && is_line_detected_) {
                exit_line_seen_ = true;
                exit_line_detect_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Line detected in EXIT!");
            }

            if (exit_line_seen_ && (now - exit_line_detect_time_).seconds() > 1.0) {
                RCLCPP_INFO(this->get_logger(), "Stable after line. Resuming Following.");
                pid_controller_.reset();
                target_yaw_ = current_yaw_; 
                current_state_ = State::CORRIDOR_FOLLOWING;
                exit_line_seen_ = false; // Reset pre budúcu križovatku
                return;
            }

            // Timeout ak čiara nie je
            if (!exit_line_seen_ && (now - detection_time_).seconds() > 4.0) {
                 RCLCPP_WARN(this->get_logger(), "Exit timeout. Resuming.");
                 pid_controller_.reset();
                 target_yaw_ = current_yaw_; 
                 current_state_ = State::CORRIDOR_FOLLOWING;
                 exit_line_seen_ = false;
                 return;
            }
        }
        break;
        default: break;
    }
       
    if (current_state_ != State::TURNING && current_state_ != State::EXIT_CORNER && current_state_ != State::CALIBRATION) {
        // --- Výpočet PWM (inspirováno _better pro hladší chod) ---
        final_correction = std::clamp(final_correction, -max_correction_, max_correction_);
        
        uint8_t l_speed = static_cast<uint8_t>(std::clamp(base_speed_ - final_correction, 0.0f, 255.0f));
        uint8_t r_speed = static_cast<uint8_t>(std::clamp(base_speed_ + final_correction, 0.0f, 255.0f));
        
        publish_motor_command(l_speed, r_speed);
        // --- GLOBAL STATE LOG ---
    }
    if (should_log) {
        std::string state_str = "UNKNOWN";
        switch (current_state_) {
            case State::CALIBRATION:       state_str = "CALIBRATION"; break;
            case State::CORRIDOR_FOLLOWING: state_str = "FOLLOWING"; break;
            case State::ONE_WALL_FOLLOWING: state_str = "ONE WALL"; break;
            case State::INTERSECTION_STRAIGHT: state_str = "INTERSECTION"; break;
            case State::TURNING:           state_str = "TURNING"; break;
            case State::EXIT_CORNER:       state_str = "EXIT"; break;
        }
        RCLCPP_INFO(this->get_logger(), "CURRENT STATE: %s", state_str.c_str());
    }
    }

}