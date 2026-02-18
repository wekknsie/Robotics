
#pragma once

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>

class RosExampleClass {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    RosExampleClass(const rclcpp::Node::SharedPtr &node, const std::string &topic, double freq)
        : node_(node), start_time_(node_->now()) {

        // Initialize the publisher
        publisher_ = node_->create_publisher<std_msgs::msg::Float32>(topic, 1);

        // Initialize the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            topic, 1, std::bind(&RosExampleClass::subscriber_callback, this, std::placeholders::_1));

        // Create a timer
        if(freq == 1){
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
                std::bind(&RosExampleClass::timer_callback, this));
        }else{
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
                std::bind(&RosExampleClass::timer_callback2, this));
        }
        

        RCLCPP_INFO(node_->get_logger(), "Node setup complete for topic: %s", topic.c_str());
    }

private:
    void timer_callback() {
        RCLCPP_INFO(node_->get_logger(), "Timer triggered. Publishing uptime...");

        double uptime = (node_->now() - start_time_).seconds();
        
        double result = (std::sin(uptime) + 1) * 500;
        publish_message(result);
    }
    void timer_callback2() {
        RCLCPP_INFO(node_->get_logger(), "Timer triggered. Publishing uptime...");

        double uptime = (node_->now() - start_time_).seconds();
        double result = (std::cos(uptime) + 1) * 500;
        publish_message(result);
    }
    void subscriber_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received: %f", msg->data);
    }

    void publish_message(float value_to_publish) {
        auto msg = std_msgs::msg::Float32();
        msg.data = value_to_publish;
        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published: %f", msg.data);
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publisher, subscriber, and timer
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;
};

