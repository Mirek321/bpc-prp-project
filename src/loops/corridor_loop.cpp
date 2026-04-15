#include "loops/corridor_loop.hpp"
#include <algorithm>  // pre std::clamp

namespace nodes {

CorridorLoopNode::CorridorLoopNode(const rclcpp::NodeOptions& options)
    : Node("corridor_loop_node", options),
      // ← PID parametre: KP, KI, KD (iné ako pre line following!)
      // Podľa prednášky 6: začni s P-only, potom pridaj I/D
      pid_controller_(8.0f, 0.0f, 0.0f) {
    
    // 1) Subscribe na error z lidar_node
    // (podobné ako bpc_prp_robot/line_error z prednášky 6)
    error_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar",  // ← téma, ktorú publishuje lidar_node
        10,
        std::bind(&CorridorLoopNode::error_callback, this, std::placeholders::_1)
    );
    
    // 2) Publish motor commands (kompatibilné s bpc_prp_robot)
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "bpc_prp_robot/motor_commands",  // ← rovnaká téma ako LineLoopNode
        10
    );
    
    // 3) Inicializácia času
    last_callback_time_ = this->now();
    
    // 4) Control loop timer (50 Hz = 20 ms – odporúčané pre stabilnú jazdu)
    // ⚠️ Tvoj LineLoopNode má 1 ms (1000 Hz) – to je veľmi rýchle, môže spôsobovať jitter
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),  // ← 50 Hz
        std::bind(&CorridorLoopNode::control_loop_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "CorridorLoopNode started @ 50Hz");
}

void CorridorLoopNode::error_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // Ulož novú chybu: error = left_wall - right_wall
    // (presne ako pri line following: error = left_IR - right_IR)
    // Podľa prednášky 6 (Line Detection - Differential Approach)
    current_error_ = msg->data;
    has_new_error_ = true;
}

void CorridorLoopNode::control_loop_callback() {
    // 1) Výpočet dt (čas od posledného kroku)
    rclcpp::Time now = this->now();
    double dt = (now - last_callback_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {  // ochrana proti skoku času
        dt = 0.02;  // fallback na 50 Hz
    }
    last_callback_time_ = now;
    
    // 2) Ak nemáme novú chybu, preskoč (voliteľné)
    if (!has_new_error_) {
        return;
    }
    has_new_error_ = false;
    
    // 3) PID výpočet (prednáška 6 - PID Control)
    // error > 0 = robot je vpravo (ľavá stena je ďalej) → zatoč vľavo
    // error < 0 = robot je vľavo (pravá stena je ďalej) → zatoč vpravo
    float correction = pid_controller_.step(current_error_, dt);
    
    // 4) Orezanie korekcie (aby sa robot neotočil na mieste)
    correction = std::clamp(correction, -max_correction_, max_correction_);
    
    // 5) Výpočet rýchlostí kolies (podobné ako LineLoopNode)
    // Podľa prednášky 5 (Inverse Kinematics):
    // ωL = (2*v - ω*L) / (2*R)
    // ωR = (2*v + ω*L) / (2*R)
    // Ale v našom prípade máme už "PWM hodnoty", takže zjednodušíme:
    float left_speed_float = base_speed_ + correction;
    float right_speed_float = base_speed_ - correction;
    
    // 6) Orezanie rýchlostí na platný rozsah
    // ⚠️ Pre chodbu použi 0-255 (nie 127-255), aby sa robot vedel úplne zastaviť pri korekcii
    left_speed_float = std::clamp(left_speed_float, 0.0f, 255.0f);
    right_speed_float = std::clamp(right_speed_float, 0.0f, 255.0f);
    
    uint8_t left_speed = static_cast<uint8_t>(left_speed_float);
    uint8_t right_speed = static_cast<uint8_t>(right_speed_float);
    
    // 7) Debug log (každých 20 iterácií = ~2.5x za sekundu pri 50 Hz)
    if (++log_counter_ % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), 
                    "Speed: L=%d R=%d | Error=%.2f | Corr=%.1f", 
                    left_speed, right_speed, current_error_, correction);
    }
    
    // 8) Publish motor command
    publish_motor_command(left_speed, right_speed);
}

void CorridorLoopNode::publish_motor_command(uint8_t left, uint8_t right) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.push_back(left);
    msg.data.push_back(right);
    motor_publisher_->publish(msg);
}

}  // namespace nodes