#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"

#include "./nodes/io_node.hpp"
#include "./nodes/led_publisher.hpp"
#include "shared_place.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto state = std::make_shared<SharedState>();

    auto subscriber = std::make_shared<MinimalSubscriber>(state);
    auto publisher = std::make_shared<MinimalPublisher>(state);

    // Add nodes to the executor
    executor->add_node(subscriber);
    executor->add_node(publisher);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
