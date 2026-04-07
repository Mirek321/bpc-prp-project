#pragma once
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "algorithms/lidar_filter.hpp"

namespace nodes {
class LidarNode : public rclcpp::Node {
public:
    explicit LidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LidarNode() override = default;
    
    int get_value() const;

private:
    int value_ = -1;
    int log_counter_ = 0;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_publisher_;
    
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};
}  // namespace nodes