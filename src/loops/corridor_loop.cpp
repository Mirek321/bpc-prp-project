#include "loops/corridor_loop.hpp"
#include <algorithm>  
#include <cmath>

namespace nodes {

CorridorLoopNode::CorridorLoopNode(const rclcpp::NodeOptions& options)
    : Node("corridor_loop_node", options),
      pid_controller_(8.0f, 0.0f, 0.0f) {
    
    // 1) Odběratelé dat
    error_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar", 10,
        std::bind(&CorridorLoopNode::error_callback, this, std::placeholders::_1));
    
    front_dist_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/front_distance", 10,
        std::bind(&CorridorLoopNode::front_dist_callback, this, std::placeholders::_1));

    side_dist_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "bpc_prp_robot/side_distances", 10,
        std::bind(&CorridorLoopNode::side_dist_callback, this, std::placeholders::_1));
    
    yaw_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/yaw", 10,
        std::bind(&CorridorLoopNode::yaw_callback, this, std::placeholders::_1));
    
    imu_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "bpc_prp_robot/imu_ready", 10,
        std::bind(&CorridorLoopNode::imu_ready_callback, this, std::placeholders::_1));

    // 2) Vydavatel příkazů pro motory
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "bpc_prp_robot/motor_commands", 10);

    // 3) Inicializace času a stavu
    last_callback_time_ = this->now();
    last_state_change_time_ = this->now();
    current_state_ = State::CALIBRATION;
    
    // 4) Timer 50 Hz
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CorridorLoopNode::control_loop_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "CorridorLoopNode started @ 50Hz");
}

void CorridorLoopNode::front_dist_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_front_distance_ = msg->data;
}

void CorridorLoopNode::side_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        current_left_dist_ = msg->data[0];
        current_right_dist_ = msg->data[1];
    }
}

void CorridorLoopNode::yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_yaw_ = msg->data;
}

void CorridorLoopNode::error_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_error_ = msg->data;
    has_new_error_ = true;
}

void CorridorLoopNode::control_loop_callback() {
    rclcpp::Time now = this->now();
    double dt = (now - last_callback_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) dt = 0.02;
    last_callback_time_ = now;

    // --- EMERGENCY STOP / REAKCE NA STĚNU ---
    if (current_state_ != State::CALIBRATION && current_front_distance_ < STOP_DISTANCE) {
        if (current_state_ == State::DRIVE_TO_CENTER || current_state_ == State::CORRIDOR_FOLLOWING) {
            RCLCPP_WARN(this->get_logger(), "Obstacle! Emergency turn.");
            float turn_direction = (current_error_ > 0) ? -1.0f : 1.0f;
            target_yaw_ = target_yaw_ + (turn_direction * M_PI / 2.0);
            
            // Normalizace cílového úhlu
            while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
            while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
            
            current_state_ = State::TURNING;
            return;
        }
        publish_motor_command(127, 127);
        return;
    }

    switch (current_state_) {
        case State::CALIBRATION:
            if (is_imu_ready_) {
                current_state_ = State::CORRIDOR_FOLLOWING;
                last_state_change_time_ = now;
                RCLCPP_INFO(this->get_logger(), "IMU Ready.");
            }
            break;

              case State::CORRIDOR_FOLLOWING:
        {
            // 1️⃣ Nastav target_yaw_ IBA RAZ pri vstupe do stavu
            static bool yaw_ref_set = false;
            if (!yaw_ref_set) {
                target_yaw_ = current_yaw_;
                yaw_ref_set = true;
                RCLCPP_INFO(this->get_logger(), "🎯 Target yaw set: %.3f", target_yaw_);
            }

            // 2️⃣ Výpočet deviácie + normalizácia
            float yaw_deviation = target_yaw_ - current_yaw_;
            while (yaw_deviation > static_cast<float>(M_PI)) yaw_deviation -= 2.0f * static_cast<float>(M_PI);
            while (yaw_deviation < -static_cast<float>(M_PI)) yaw_deviation += 2.0f * static_cast<float>(M_PI);

            // 3️⃣ Deadband pre yaw (odstráni mikro-kmitanie)
            if (std::abs(yaw_deviation) < 0.04f) yaw_deviation = 0.0f;

            // ⛔ VYMAŽ ADAPTÍVNY target_yaw_ BLOK (ak ho tam ešte máš!)
            // if (std::abs(current_error_) > 0.05f) { ... } ← ODSTRÁŇ CELÝ IF

            // 4️⃣ Filter pre LiDAR
            filtered_error_ = ERROR_ALPHA * current_error_ + (1.0f - ERROR_ALPHA) * filtered_error_;
            if (std::abs(filtered_error_) < ERROR_DEADBAND) filtered_error_ = 0.0f;

            // 5️⃣ Korekcie (ZNÍŽENÉ GAINY)
            float lidar_correction = pid_controller_.step(filtered_error_, static_cast<float>(dt));
            float imu_correction   = yaw_deviation * 1.0f; // ← YAW_P_GAIN znížený z 2.5 na 1.0

            // 6️⃣ Mäkší clamp pre koridor (±80 je na ostré zatáčky, nie na nasledovanie)
            const float CORRIDOR_MAX_COR = 35.0f; 
            float total_correction = std::clamp(lidar_correction + imu_correction, -CORRIDOR_MAX_COR, CORRIDOR_MAX_COR);

            // 7️⃣ Aplikácia na motory
            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ + total_correction, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ - total_correction, 0.0f, 255.0f))
            );

            // Debug
            RCLCPP_INFO(this->get_logger(), 
                "Err: %.3f | YawErr: %.3f | LidarCor: %.1f | YawCor: %.1f | Total: %.1f",
                filtered_error_, yaw_deviation, lidar_correction, imu_correction, total_correction);
        }
        break;

        case State::DRIVE_TO_CENTER:
        {
            const float WALL_IS_NEAR = 0.325f; 
            bool has_data = (current_left_dist_ > 0.001f || current_right_dist_ > 0.001f);

            // Pokud se robot omylem ocitne mezi stěnami, zruší najíždění do středu
            if (has_data && current_left_dist_ < WALL_IS_NEAR && current_right_dist_ < WALL_IS_NEAR) {
                RCLCPP_WARN(this->get_logger(), "False corner detection! Returning.");
                current_state_ = State::CORRIDOR_FOLLOWING;
                last_state_change_time_ = now;
                return;
            }

            if ((now - detection_time_).seconds() < 1.75f) {
                float yaw_error = target_yaw_ - current_yaw_;
                while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

                float correction = yaw_error * 25.0f;
                publish_motor_command(
                    static_cast<uint8_t>(std::clamp(base_speed_ - correction, 0.0f, 255.0f)),
                    static_cast<uint8_t>(std::clamp(base_speed_ + correction, 0.0f, 255.0f))
                );
            } else {
                // Přepočet cíle pro 90° zatáčku
                target_yaw_ = target_yaw_ + (last_turn_direction_ * M_PI / 2.0);
                while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
                while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;

                RCLCPP_INFO(this->get_logger(), "At center. Turning...");
                current_state_ = State::TURNING;
            }
        }
        break;

        case State::TURNING:
        {
            float yaw_error = target_yaw_ - current_yaw_;
            while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            if (std::abs(yaw_error) < YAW_PRECISION) {
                RCLCPP_INFO(this->get_logger(), "Turn complete.");
                detection_time_ = now; 
                current_state_ = State::EXIT_CORNER;
            } else {
                uint8_t power = 12; 
                if (yaw_error > 0) publish_motor_command(127 - power, 127 + power);
                else publish_motor_command(127 + power, 127 - power);
            }
        }
        break;
        
        case State::EXIT_CORNER:
        {
            double time_in_exit = (now - detection_time_).seconds();
            
            // 1) Řízení pohybu (stále držíme směr podle IMU)
            float yaw_error = target_yaw_ - current_yaw_;
            while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            float correction = yaw_error * 25.0f;
            publish_motor_command(
                static_cast<uint8_t>(std::clamp(base_speed_ - correction, 0.0f, 255.0f)),
                static_cast<uint8_t>(std::clamp(base_speed_ + correction, 0.0f, 255.0f))
            );

            // 2) Logika ukončení stavu (Terminační podmínka)
            const float IN_CORRIDOR_THRESHOLD = 3.80f; // Vzdálenost, kdy už jsme v chodbě
            bool has_data = (current_left_dist_ > 0.001f && current_right_dist_ > 0.001f);
            
            // Podmínka pro ukončení:
            // Musíme tam být aspoň 0.5s (prevence záškubů) 
            // A ZÁROVEŇ musíme vidět stěny z obou stran (jsme v chodbě)
            // NEBO uplynul maximální bezpečnostní čas (např. 3s), kdyby lidar selhal
            if (time_in_exit > 1.9f) {
                if ((has_data && current_left_dist_ < IN_CORRIDOR_THRESHOLD && current_right_dist_ < IN_CORRIDOR_THRESHOLD) 
                    || time_in_exit > 3.0f) {
                    
                    RCLCPP_INFO(this->get_logger(), "New corridor confirmed by Lidar. Switching to Following.");
                    last_state_change_time_ = now; 
                    current_state_ = State::CORRIDOR_FOLLOWING;
                }
            }
        }
        break;

                default: break;
    }
}

void CorridorLoopNode::publish_motor_command(uint8_t left, uint8_t right) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.push_back(left);
    msg.data.push_back(right);
    motor_publisher_->publish(msg);
}

} // namespace nodes