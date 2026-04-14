#include "nodes/camera_node.hpp"

namespace nodes {

CameraNode::CameraNode(const rclcpp::NodeOptions& options)
: Node("camera_node", options)
{
    camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed",
        1,
        std::bind(&CameraNode::on_camera_image, this, std::placeholders::_1)  
    );

    markers_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "bpc_prp_robot/arucos_detected",
        10
    );

    RCLCPP_INFO(this->get_logger(), "CameraNode started. Listening to /bpc_prp_robot/camera/compressed");
}

void CameraNode::on_camera_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        return;
    }

    if (!cv_ptr || cv_ptr->image.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Empty image after decompression");
        return;
    }

    auto markers = detector_.detect(cv_ptr->image);

    std_msgs::msg::Int16MultiArray result_msg;
    for (const auto& marker : markers) {
        result_msg.data.push_back(static_cast<int16_t>(marker.id));  
    }
    markers_pub_->publish(result_msg);
}

} // namespace nodes