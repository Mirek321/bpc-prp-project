#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/camera_node.hpp"
#include "loops/line_loop.hpp"
#include "loops/corridor_loop.hpp"
#include "loops/maze_loop.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "rviz_example_class.hpp"
#include <thread>     
#include <vector>  
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto io_node = std::make_shared<nodes::IoNode>(options);
    auto motor_node = std::make_shared<nodes::MotorNode>(options);
    auto line_node = std::make_shared<nodes::LineNode>(options);
    auto lidar_node = std::make_shared<nodes::LidarNode>(options);
    auto maze_loop_node = std::make_shared<nodes::MazeLoopNode>(options);
    auto camera_node = std::make_shared<nodes::CameraNode>(options);
    auto imu_node = std::make_shared<nodes::ImuNode>(options);

    auto io_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto motor_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto line_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto camera_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto lidar_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto maze_loop_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto imu_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    io_executor->add_node(io_node);
    motor_executor->add_node(motor_node);
    line_executor->add_node(line_node);
    lidar_executor->add_node(lidar_node);
    maze_loop_executor->add_node(maze_loop_node);
    camera_executor->add_node(camera_node);
    imu_executor->add_node(imu_node);


    std::vector<std::thread> threads;
    
    threads.emplace_back([motor_executor](){
        motor_executor->spin();
    });

    threads.emplace_back([io_executor](){
        io_executor->spin();
    });

    threads.emplace_back([line_executor](){
        line_executor->spin();
    });
    threads.emplace_back([lidar_executor](){
        lidar_executor->spin();
    });
    threads.emplace_back([maze_loop_executor](){
        maze_loop_executor->spin();
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

    rclcpp::shutdown();

    motor_executor->cancel();
    line_executor->cancel();
    lidar_executor->cancel();
    maze_loop_executor->cancel();
    camera_executor->cancel();
    imu_executor->cancel();

    for(auto& t: threads){
        t.join();
    }

    return 0;
}
