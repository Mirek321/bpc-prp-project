#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>

#define MIN_L_VALUE 0
#define MAX_L_VALUE 893
#define MIN_R_VALUE 0
#define MAX_R_VALUE 1023

enum class DiscreteLinePose {
    LineOnLeft = 0,
    LineOnRight = 1,
    LineNone = 2,
    LineBoth = 3,
};

typedef struct {
    int left_min_value;
    int left_max_value;
    int right_min_value;
    int right_max_value;
} LineSensorCalibration;

namespace nodes {
    class LineNode : public rclcpp::Node {
    public:
        explicit LineNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~LineNode() override = default;

        // Relative pose to line [m]
        float get_continuous_line_pose() const;
        float current_continuous = 0;
        DiscreteLinePose current_discrete;

        DiscreteLinePose get_discrete_line_pose() const;

    private:
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pose_publisher_; 
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_publisher_; 
        rclcpp::Time last_process_time_;
        float threshold_l = 0.7;
        float threshold_r = 0.4;
        LineSensorCalibration calibration_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };
}