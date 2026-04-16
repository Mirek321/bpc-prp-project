#include "loops/corridor_loop.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

CorridorLoopNode::CorridorLoopNode(const rclcpp::NodeOptions& options)
    : Node("corridor_loop_node", options),
      pid_controller_(9.5f, 0.0f, 0.0f) // P-only (bpc-prp-6-pid.pdf)
{
    error_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar", 10,
        std::bind(&CorridorLoopNode::error_callback, this, std::placeholders::_1));
    
    front_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/front_distance", 10,
        std::bind(&CorridorLoopNode::front_dist_callback, this, std::placeholders::_1));

    side_dist_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "bpc_prp_robot/side_distances", 10,
        std::bind(&CorridorLoopNode::side_dist_callback, this, std::placeholders::_1));
    
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/yaw", 10,
        std::bind(&CorridorLoopNode::yaw_callback, this, std::placeholders::_1));
    
    imu_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/imu_ready", 10,
        std::bind(&CorridorLoopNode::imu_ready_callback, this, std::placeholders::_1));

    line_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/line_detected", 
        10,
        std::bind(&CorridorLoopNode::line_detected_callback, this, std::placeholders::_1));
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "bpc_prp_robot/motor_commands", 10);
        


    last_callback_time_ = this->now();
    hole_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    current_state_ = State::CALIBRATION;
    
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50 Hz
        std::bind(&CorridorLoopNode::control_loop_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "CorridorLoopNode started @ 50Hz");
}

void CorridorLoopNode::front_dist_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_front_distance_ = msg->data; }
void CorridorLoopNode::side_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) { current_left_dist_ = msg->data[0]; current_right_dist_ = msg->data[1]; }
}
void CorridorLoopNode::yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_yaw_ = msg->data; }
void CorridorLoopNode::error_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_error_ = msg->data; has_new_error_ = true; }
void CorridorLoopNode::imu_ready_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!is_imu_ready_ && msg->data) RCLCPP_INFO(this->get_logger(), "IMU Calibration finished - starting movement!");
    is_imu_ready_ = msg->data;
}
void CorridorLoopNode::line_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_line_detected_ = msg->data;
}
float CorridorLoopNode::normalize_angle(float angle) const {
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void CorridorLoopNode::publish_motor_command(uint8_t left, uint8_t right) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.push_back(left); msg.data.push_back(right);
    motor_pub_->publish(msg);
}

void CorridorLoopNode::control_loop_callback() {
    rclcpp::Time now = this->now();
    double dt = (now - last_callback_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) dt = 0.02;
    last_callback_time_ = now;

    // Emergency stop (bezpečnostný základ)
    if (current_state_ != State::CALIBRATION && current_front_distance_ < STOP_DISTANCE) {
        publish_motor_command(127, 127);
        return;
    }

    switch (current_state_) {
        case State::CALIBRATION:
            if (is_imu_ready_) {
                target_yaw_ = current_yaw_;
                current_state_ = State::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "🚀 IMU Ready. Starting corridor following.");
            }
            break;

               case State::CORRIDOR_FOLLOWING:
        {
            // 1️⃣ Konštanty
            const float MIN_LIDAR   = 0.16f;  // Slepá zóna LiDARu
            const float MAX_FOLLOW  = 1.0f;   // Maximálna vzdialenosť pre nasledovanie steny
            const float OPEN_DIST   = 0.40f;  // Vzdialenosť, od ktorej je strana "otvorená"
            const float DESIRED     = 0.20f;  // Cieľový odstup od steny pri centrovaní

            // 2️⃣ Validita steny pre nasledovanie (blízko, ale nie v slepej zóne)
            bool L_valid = (current_left_dist_ > MIN_LIDAR && current_left_dist_ < MAX_FOLLOW);
            bool R_valid = (current_right_dist_ > MIN_LIDAR && current_right_dist_ < MAX_FOLLOW);

            // 3️⃣ Detekcia otvoreného priestoru (strana je "open")
            bool L_open = (current_left_dist_ > OPEN_DIST);
            bool R_open = (current_right_dist_ > OPEN_DIST);
            bool front_open = (current_front_distance_ > OPEN_DIST);
            bool front_close = (current_front_distance_ < 30);

            // 4️⃣ Laterálna chyba: LEN ak sú OBE steny validné (robot je v chodbe)
            float raw_err = 0.0f;
            if (L_valid && R_valid) {
                // Centrovanie v chodbe
                raw_err = (current_right_dist_ - current_left_dist_) / 2.0f;
            }
            else if (L_valid && !R_valid) {
                // ⚠️ Len ľavá stena je blízko -> Drž odstup DESIRED od ľavej
                // Aby sa robot netočil do prázdna, chyba je relatívna k desired offsetu
                raw_err = DESIRED_HALF_WIDTH - current_left_dist_; 
            } 
            else if (!L_valid && R_valid) {
                // ⚠️ Len pravá stena je blízko -> Drž odstup DESIRED od pravej
                raw_err = current_right_dist_ - DESIRED_HALF_WIDTH;
            } 
            // else: jedna alebo obe steny chýbajú → raw_err = 0.0f (čistý IMU hold)

            // 5️⃣ Filter + Deadband
            filtered_error_ = ERROR_ALPHA * raw_err + (1.0f - ERROR_ALPHA) * filtered_error_;
            if (std::abs(filtered_error_) < ERROR_DEADBAND) filtered_error_ = 0.0f;

            // 6️⃣ IMU yaw deviácia
            float yaw_dev = normalize_angle(target_yaw_ - current_yaw_);
            if (std::abs(yaw_dev) < YAW_DEADBAND) yaw_dev = 0.0f;

            // 7️⃣ Korekcie: LiDAR + IMU
            float lidar_cor = pid_controller_.step(filtered_error_, static_cast<float>(dt));
            float imu_cor   = yaw_dev * YAW_P_GAIN;
            float total_cor = std::clamp(lidar_cor + imu_cor, -max_correction_, max_correction_);

            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ - total_cor, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ + total_cor, 0.0f, 255.0f))
            );

            // 8️⃣ Detekcia križovatky / "diery" s hysterézou
            // Podmienka: predok otvorený + aspoň jedna strana je open
            bool hole = front_close && (L_open || R_open);
            
            if (hole && (!L_open && R_open) || (L_open && !R_open)) {
                if ((now - hole_start_time_).seconds() > 1.0) {
                    current_state_ = State::DRIVE_TO_CENTER;
                    detection_time_ = now;
                    // Smer otáčania: ak je vľavo open, točíme vľavo (-1), inak vpravo (+1)
                    last_turn_direction_ = L_open ? -1.0f : 1.0f;
                    target_yaw_ = current_yaw_;
                    pid_controller_.reset();
                RCLCPP_INFO(this->get_logger(), 
                "L:%.2f(%s%s) R:%.2f(%s%s) F:%.2f(%s) | Err:%.3f | Yaw:%.3f | Tot:%.1f",
                current_left_dist_, 
                    L_valid ? "V" : "-", L_open ? "O" : "",
                current_right_dist_, 
                    R_valid ? "V" : "-", R_open ? "O" : "",
                current_front_distance_,
                    front_close ? "C" : "O", 
                filtered_error_, yaw_dev, total_cor);
                    RCLCPP_INFO(this->get_logger(), "🚪 Intersection! Turn %s", 
                                L_open ? "LEFT" : "RIGHT");
                }
            } else {
                hole_start_time_ = now; // Reset timer ak "diera" zmizne
            }

            // Debug: (V=valid pre following, O=open priestor, X=neviditeľná/ďaleko)
            RCLCPP_INFO(this->get_logger(), 
                "L:%.2f(%s%s) R:%.2f(%s%s) F:%.2f(%s) | Err:%.3f | Yaw:%.3f | Tot:%.1f",
                current_left_dist_, 
                    L_valid ? "V" : "-", L_open ? "O" : "",
                current_right_dist_, 
                    R_valid ? "V" : "-", R_open ? "O" : "",
                current_front_distance_,
                    front_close ? "C" : "O", 
                filtered_error_, yaw_dev, total_cor);
        }
        break;

        case State::DRIVE_TO_CENTER:
        {
            // Krátke vyrovnanie do stredu križovatky (0.8 s)
            float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
            float corr = std::clamp(yaw_err * 30.0f, -40.0f, 40.0f);
            publish_motor_command(static_cast<uint8_t>(std::clamp(base_speed_ - corr, 0.0f, 255.0f)),
                                  static_cast<uint8_t>(std::clamp(base_speed_ + corr, 0.0f, 255.0f)));

            if ((now - detection_time_).seconds() > 0.8) {
                target_yaw_ = normalize_angle(current_yaw_ + (last_turn_direction_ * (M_PI_2)));
                current_state_ = State::TURNING;
                RCLCPP_INFO(this->get_logger(), "🎯 Target yaw: %.3f rad", target_yaw_);
            }
        }
        break;

        case State::TURNING:
        {
            // P-regulácia na IMU yaw (bpc-prp-6-pid.pdf: "Start with P-only Control")
            float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
            float corr = std::clamp(yaw_err * K_TURN_P, -TURN_MAX_PWM, TURN_MAX_PWM);
            publish_motor_command(static_cast<uint8_t>(std::clamp(127.0f - corr, 0.0f, 255.0f)),
                                  static_cast<uint8_t>(std::clamp(127.0f + corr, 0.0f, 255.0f)));
            RCLCPP_INFO(this->get_logger(), "🎯 Target yaw: %.3f %.3f rad", current_yaw_, target_yaw_);
            if (std::abs(yaw_err) < YAW_PRECISION) {
                publish_motor_command(127, 127);
                current_state_ = State::EXIT_CORNER;
                detection_time_ = now;
                target_yaw_ = current_yaw_;
                RCLCPP_INFO(this->get_logger(), "✅ Turn complete. Exiting.");
                
            }
        }
        break;

           case State::EXIT_CORNER:
        {
            // 1️⃣ Drž smer podľa IMU (target_yaw_ je nastavený po dokončení TURNING)
            float yaw_err = normalize_angle(target_yaw_ - current_yaw_);
            
            // P-korekcia pre udržanie smeru (stačí menší zisk, napr. 20-30)
            float imu_cor = yaw_err * 25.0f; 
            
            // Obmedzenie korekcie, aby robot nekrútil príliš ostro
            float total_cor = std::clamp(imu_cor, -40.0f, 40.0f);

            // Jazda rovno s jemnou IMU korekciou
            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ + total_cor, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ - total_cor, 0.0f, 255.0f))
            );

            // 2️⃣ Detekcia návratu do chodby
            // Chodba znamená, že vidíme steny na oboch stranách v rozumnej vzdialenosti
            // Napr. menej ako 1.5 metra (prispôsob šírke tvojej trate)
            bool left_wall_seen = (current_left_dist_ > 0.1f && current_left_dist_ < 1.5f);
            bool right_wall_seen = (current_right_dist_ > 0.1f && current_right_dist_ < 1.5f);
            
            // Podmienka: Musíme vidieť obe steny A byť vyrovnaní (malá yaw chyba)
            if (left_wall_seen && right_wall_seen && std::abs(yaw_err) < 0.1f) {
                
                // Voliteľné: Počkaj chvíľu, aby si bol stabilný (napr. 0.5s)
                // Alebo prepni okamžite, ak chceš rýchlu odozvu
                if ((now - detection_time_).seconds() > 2) {
                    RCLCPP_INFO(this->get_logger(), "🟢 Walls detected. Resuming Corridor Following.");
                    
                    // Dôležité: Resetuj PID a cieľový uhol na aktuálny stav,
                    // aby nasledovanie chodby začalo "čisto"
                    pid_controller_.reset();
                    target_yaw_ = current_yaw_; 
                    
                    current_state_ = State::CORRIDOR_FOLLOWING;
                }
            }
            
            // Bezpečnostná poistka: Ak by LiDAR zlyhal, po 3 sekundách pokračuj anyway
            if ((now - detection_time_).seconds() > 3.0) {
                 RCLCPP_WARN(this->get_logger(), "⚠️ Exit timeout. Forcing Corridor Following.");
                 pid_controller_.reset();
                 target_yaw_ = current_yaw_;
                 current_state_ = State::CORRIDOR_FOLLOWING;
            }
        }
        break;

        default: break;
    }
}

} // namespace nodes