#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"

#include "./nodes/io_node.hpp"
#include "./nodes/led_publisher.hpp"
#include "./nodes/motor_node.hpp"
#include "./nodes/line_node.hpp"
#include "./nodes/lidar_node.hpp"
#include "shared_place.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto state = std::make_shared<SharedState>();
    state->last_button = 0;

    auto buttonNode = std::make_shared<ButtonNode>(state);
    auto ledNode = std::make_shared<LedNode>(state);
    auto motorNode = std::make_shared<MotorNode>(state);
    auto lineNode = std::make_shared<LineNode>(state);
    auto lidarNode = std::make_shared<LidarNode>(state);

    // Add nodes to the executor
    executor->add_node(buttonNode);
    executor->add_node(ledNode);
    executor->add_node(motorNode);
    executor->add_node(lineNode);
    executor->add_node(lidarNode);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
