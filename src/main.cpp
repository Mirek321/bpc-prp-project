#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "rviz_example_class.hpp"
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Vytvorenie IoNode
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();
    auto node = std::make_shared<RvizExampleClass>("rviz_topic", 30.0);

    // Pridanie do executor-a
    executor->add_node(motor_node);
    executor->add_node(io_node);
    executor->add_node(node);

    RCLCPP_INFO(io_node->get_logger(), "IoNode running...");
    RCLCPP_INFO(motor_node->get_logger(), "MotorNode running...");

    motor_node->set_speed(150,150);
    // Shutdown ROS 2

     executor->spin();

    rclcpp::shutdown();
    return 0;
}
