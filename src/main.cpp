#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/camera_node.hpp"
#include "loops/line_loop.hpp"
#include "loops/corridor_loop.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "rviz_example_class.hpp"
#include <thread>     
#include <vector>  
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto motor_node = std::make_shared<nodes::MotorNode>(options);
    auto line_node = std::make_shared<nodes::LineNode>(options);
    // auto loop_line_node = std::make_shared<nodes::LineLoopNode>(options);
    auto lidar_node = std::make_shared<nodes::LidarNode>(options);
    auto corridor_loop_node = std::make_shared<nodes::CorridorLoopNode>(options);
    auto camera_node = std::make_shared<nodes::CameraNode>(options);
    auto imu_node = std::make_shared<nodes::ImuNode>(options);

    auto motor_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto line_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // auto loop_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto camera_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto lidar_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto corridor_loop_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto imu_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    motor_executor->add_node(motor_node);
    line_executor->add_node(line_node);
    // loop_executor->add_node(loop_line_node);
    lidar_executor->add_node(lidar_node);
    corridor_loop_executor->add_node(corridor_loop_node);
    camera_executor->add_node(camera_node);
    imu_executor->add_node(imu_node);

    // RCLCPP_INFO(loop_line_node->get_logger(), "Starting 3 nodes on 3 separate threads...");

    std::vector<std::thread> threads;
    
    threads.emplace_back([motor_executor](){
        motor_executor->spin();
    });

    threads.emplace_back([line_executor](){
        line_executor->spin();
    });
    // threads.emplace_back([loop_executor](){
    //     loop_executor->spin();
    // });

    threads.emplace_back([lidar_executor](){
        lidar_executor->spin();
    });
       threads.emplace_back([corridor_loop_executor](){
        corridor_loop_executor->spin();
    });
    threads.emplace_back([camera_executor](){
        camera_executor->spin();
    });
    threads.emplace_back([imu_executor](){
        imu_executor->spin();
    });

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // RCLCPP_INFO(loop_line_node->get_logger(),"Shutting down... ");
    rclcpp::shutdown();

    motor_executor->cancel();
    line_executor->cancel();
    // loop_executor->cancel();
    lidar_executor->cancel();
    corridor_loop_executor->cancel();
    camera_executor->cancel();
    imu_executor->cancel();

    for(auto& t: threads){
        t.join();
    }

    // RCLCPP_INFO(loop_line_node->get_logger(), "All threads joined. Exit.");
    return 0;
}
