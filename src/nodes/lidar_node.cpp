#include "nodes/lidar_node.hpp"

namespace nodes {

LidarNode::LidarNode(const rclcpp::NodeOptions& options) 
    : Node("lidar_node", options) {
    
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "bpc_prp_robot/lidar",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarNode::callback, this, std::placeholders::_1)
    );
    
    error_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "bpc_prp_robot/error_lidar",
        10
    );

    side_distances_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "bpc_prp_robot/side_distances", 10);

    front_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
    "bpc_prp_robot/front_distance", 10);
    
    RCLCPP_INFO(this->get_logger(), "LidarNode started. Publishing to /lidar/error");
}

int LidarNode::get_value() const {
    return value_;
}

void LidarNode::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) return;
    
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    
    auto results = filter_.apply_filter(
        ranges,
        msg->angle_min,       
        msg->angle_increment,  
        msg->range_min,
        msg->range_max
    );
    
    float error = results.left - results.right;
    
    auto error_msg = std_msgs::msg::Float32();
    error_msg.data = error;
    error_publisher_->publish(error_msg);

    auto sides_msg = std_msgs::msg::Float32MultiArray();
    sides_msg.data = {results.left, results.right, results.front_left, results.front_right};
    side_distances_publisher_->publish(sides_msg);

    auto front_msg = std_msgs::msg::Float32();
    front_msg.data = results.back;
    front_distance_publisher_->publish(front_msg);
    
    if (++log_counter_ % 20 == 0) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "L=%.2f | R=%.2f | F=%.2f | Error=%.3f",
                     results.left, results.right, results.front, error);
    }
    
    float min_dist = msg->range_max;
    for (float range : ranges) {
        if (std::isfinite(range) && range > msg->range_min && range < min_dist) {
            min_dist = range;
        }
    }
    value_ = static_cast<int>(min_dist * 1000);  
}

}  // namespace nodes