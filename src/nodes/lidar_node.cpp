#include "nodes/lidar_node.hpp"

namespace nodes {

LidarNode::LidarNode(const rclcpp::NodeOptions& options) 
    : Node("lidar_node", options) {
    
    // 1) Subscribe na surové LiDAR dáta
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "bpc_prp_robot/lidar",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarNode::callback, this, std::placeholders::_1)
    );
    
    // 2) Publish error pre corridor_loop_node (ako /line/error z prednášky 6)
    error_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar",
        10
    );
    
    RCLCPP_INFO(this->get_logger(), "LidarNode started. Publishing to /lidar/error");
}

int LidarNode::get_value() const {
    return value_;
}

void LidarNode::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) return;
    
    // 1) Preveď ranges na vector
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    
    // 2) Aplikuj filter (sektory: front, left, right, back)
    algorithms::LidarFilter filter;
    auto results = filter.apply_filter(
        ranges,
        msg->angle_min,        // angle_start
        msg->angle_increment,  // priamo z LiDAR správy
        msg->range_min,
        msg->range_max
    );
    
    // 3) Vypočítaj error (diferenciálny prístup – prednáška 6)
    // error = left_wall - right_wall
    // error > 0 = robot je vpravo → zatoč vľavo
    // error < 0 = robot je vľavo → zatoč vpravo
    float error = results.left - results.right;
    
    // 4) Publish error pre corridor_loop_node
    auto error_msg = std_msgs::msg::Float32();
    error_msg.data = error;
    error_publisher_->publish(error_msg);
    
    // 5) Debug log (každých 20 správ)
    if (++log_counter_ % 20 == 0) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "L=%.2f | R=%.2f | F=%.2f | Error=%.3f",
                     results.left, results.right, results.front, error);
    }
    
    // 6) Ulož minimálnu vzdialenosť (pre kompatibilitu)
    float min_dist = msg->range_max;
    for (float range : ranges) {
        if (std::isfinite(range) && range > msg->range_min && range < min_dist) {
            min_dist = range;
        }
    }
    value_ = static_cast<int>(min_dist * 1000);  // prevod na mm
}

}  // namespace nodes