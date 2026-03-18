#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include "shared_place.hpp"
#include "io_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

class LedNode : public rclcpp::Node
{
public:
  LedNode(std::shared_ptr<SharedState> state)
  : Node("led_node"), count_(0), state_(state)
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&LedNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::UInt8MultiArray message;

    int button_state = state_->last_button.load();
    //RCLCPP_INFO(this->get_logger(), "Button state: %d", button_state);

    if(button_state == 0) // turned off -> all LEDs are red
    {
        message.data = {
            60, 1, 1, 60, 1, 1, 60, 1, 1, 60, 1, 1
            };
    }
    else if(button_state == 1) // corridor drive -> LEDs are cycling yellow
    {
        message.data.resize(12, 0);

        fade += 3;

        if (fade >= 75) {
            fade = 0;
            cycle = (cycle + 1) % 4;
        }

        const int led_map[4] = {0, 1, 3, 2};

        int current_led = led_map[cycle];
        int next_led = led_map[(cycle + 1) % 4];

        uint8_t up = static_cast<uint8_t>(fade);
        uint8_t down = static_cast<uint8_t>(75 - fade);

        message.data[current_led * 3 + 1] = down - down / 3;
        message.data[current_led * 3] = down;
        message.data[next_led * 3 + 1] = up - up / 3;
        message.data[next_led * 3] = up;
    }
    else if(button_state == 2) // following line -> LEDs are cycling green
    {        
        message.data.resize(12, 0);

        fade += 3;

        if (fade >= 75) {
            fade = 0;
            cycle = (cycle + 1) % 4;
        }

        const int led_map[4] = {0, 1, 3, 2};

        int current_led = led_map[cycle];
        int next_led = led_map[(cycle + 1) % 4];

        uint8_t up = static_cast<uint8_t>(fade);
        uint8_t down = static_cast<uint8_t>(75 - fade);

        message.data[current_led * 3 + 1] = down;
        message.data[next_led * 3 + 1] = up;
    }

    //RCLCPP_INFO(this->get_logger(), "Publishing %zu bytes, first=%u, cycle=%d",
    //            message.data.size(), message.data.empty() ? 0 : message.data[0], cycle);

    publisher_->publish(message);
    ++count_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  size_t count_;
  int tick = 0;
  int cycle = 0;
  int fade = 0;
  bool fadeUp = true;
  std::shared_ptr<SharedState> state_;
};
