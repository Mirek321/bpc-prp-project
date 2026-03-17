#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#define MIN_L_VALUE 25
#define MAX_L_VALUE 1024
#define MIN_R_VALUE 40
#define MAX_R_VALUE 988

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
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
        LineNode();
         ~LineNode() override = default;

        // Relative pose to line [m]
        float get_continuous_line_pose() const;
        float current_continuous = 0;

        DiscreteLinePose get_discrete_line_pose() const;

    private:
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscriber_;
        rclcpp::Time last_process_time_;
        float threshold_l = 0.01;
        float threshold_r = 0.01;
        LineSensorCalibration calibration_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };
}