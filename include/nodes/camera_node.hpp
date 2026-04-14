#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp> 
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int16_multi_array.hpp> 
#include <cv_bridge/cv_bridge.h>
#include "algorithms/aruco_detector.hpp"  
#include <sstream>
namespace nodes {

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void on_camera_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr markers_pub_; 

    algorithms::ArucoDetector detector_;  
};

} 