#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "shared_place.hpp"


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(std::shared_ptr<SharedState> state)
    : Node("minimal_subscriber"), state_(state)
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/bpc_prp_robot/buttons", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

    int getButtonState()
    {
      return button_pressed_;
    }

  private:
    int button_pressed_ = -1;

    
    void topic_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Last button state: '%u'", button_pressed_);
      RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg->data);
      button_pressed_ = msg->data;
      state_->last_button = button_pressed_;
      state_->have_data.store(true);
    }
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
    std::shared_ptr<SharedState> state_;

};
