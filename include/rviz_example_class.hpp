 #pragma once

 #include <iostream>
 #include <memory>
 #include <string>
 #include <chrono>
 #include <cmath>
 #include <iomanip>
 #include <sstream>
 #include <rclcpp/rclcpp.hpp>
 #include <visualization_msgs/msg/marker_array.hpp>
 
 #define FORMAT std::fixed << std::setw(5) << std::showpos << std::setprecision(2)
 
 class RvizExampleClass : public rclcpp::Node {
 public:
     RvizExampleClass(const std::string& topic, double freq)
         : Node("rviz_example_node") // Node name in ROS 2
     {
         // Create a timer with the specified frequency (Hz)
         timer_ = this->create_wall_timer(
             std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
             std::bind(&RvizExampleClass::timer_callback, this)
         );
 
         // Create a publisher for MarkerArray messages
         markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
     }
 
 private:
     class Pose {
     public:
         Pose(float x, float y, float z) : x_{x}, y_{y}, z_{z} {}
         float x() const { return x_; }
         float y() const { return y_; }
         float z() const { return z_; }
     private:
         const float x_, y_, z_;
     };
 
     void timer_callback() {
         auto time = this->now().seconds();
         auto pose = Pose(sin(time), cos(time), 0.5 * sin(time * 3));
 
         // Create a MarkerArray message
         visualization_msgs::msg::MarkerArray msg;
         msg.markers.push_back(make_cube_marker(pose));
         msg.markers.push_back(make_text_marker(pose));
 
         // Publish the marker array
         markers_publisher_->publish(msg);
     }
 
     visualization_msgs::msg::Marker make_cube_marker(const Pose& pose) {
         visualization_msgs::msg::Marker cube;
 
         // Coordinate system
         cube.header.frame_id = "map"; // In ROS 2, "map" or "odom" is recommended
         cube.header.stamp = this->now();
 
         // Marker Type
         cube.type = visualization_msgs::msg::Marker::CUBE;
         cube.action = visualization_msgs::msg::Marker::ADD;
         cube.id = 0;
 
         // Position
         cube.pose.position.x = pose.x();
         cube.pose.position.y = pose.y();
         cube.pose.position.z = pose.z();
 
         // Orientation (Quaternion)
         cube.pose.orientation.x = 0.0;
         cube.pose.orientation.y = 0.0;
         cube.pose.orientation.z = 0.0;
         cube.pose.orientation.w = 1.0;
 
         // Size
         cube.scale.x = cube.scale.y = cube.scale.z = 0.1;
 
         // Color
         cube.color.a = 1.0; // Alpha (visibility)
         cube.color.r = 0.0;
         cube.color.g = 1.0;
         cube.color.b = 0.0;
 
         return cube;
     }
 
     visualization_msgs::msg::Marker make_text_marker(const Pose& pose) {
         visualization_msgs::msg::Marker text;
 
         // Coordinate system
         text.header.frame_id = "map";
         text.header.stamp = this->now();
 
         // Marker Type
         text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
         text.action = visualization_msgs::msg::Marker::ADD;
         text.id = 1;
 
         // Position (slightly above the cube)
         text.pose.position.x = pose.x();
         text.pose.position.y = pose.y();
         text.pose.position.z = pose.z() + 0.2;
 
         // Size
         text.scale.z = 0.1;
 
         // Text content
         std::stringstream stream;
         stream << "* Cool Cube *" << std::endl
                << "  x: " << FORMAT << pose.x() << std::endl
                << "  y: " << FORMAT << pose.y() << std::endl
                << "  z: " << FORMAT << pose.z();
         text.text = stream.str();
 
         // Color
         text.color.a = 1.0;
         text.color.r = 1.0;
         text.color.g = 1.0;
         text.color.b = 0.0;
 
         return text;
     }
 
     // ROS 2 timer
     rclcpp::TimerBase::SharedPtr timer_;
 
     // ROS 2 publisher
     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
 };
