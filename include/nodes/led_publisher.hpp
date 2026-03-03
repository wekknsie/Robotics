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
    RCLCPP_INFO(this->get_logger(), "Button state: %d", button_state);

    if(button_state == 0)
    {
        message.data = {
            20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20
            };
    }
    else if(button_state == 1)
    {
       message.data = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };
    }
    else if(button_state == 2)
    {        
        tick++;
        if(tick >= 25){
            tick = 0;
            cycle = (cycle + 1) % 8;
        }
        
        if(fadeUp){
            fade += 5;
            if(fade >= 150){
                fade = 150;
                fadeUp = false;
            }
        }else{
            fade -= 5;
            if(fade <= 0){
                fade = 0;
                fadeUp = true;
            }
        }

        switch(cycle){
            case 0:{
                message.data = {
                        fade, fade, fade, 0,0,0, 0,0,0, 0,0,0, 
                    };
                break;
                }
            case 2:
                message.data = {
                    0,0,0, 20,20,20, 0,0,0, 0,0,0 
                };
                break;
            case 4:
                message.data = {
                    0,0,0, 0,0,0, 0,0,0, 20,20,20
                };
                break;
            case 6:
                message.data = {
                    0,0,0, 0,0,0, 20,20,20, 0,0,0
                };
                break;
            default:
                message.data = {
                    0,0,0, 0,0,0, 0,0,0, 0,0,0
                };
                break;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Publishing %zu bytes, first=%u, cycle=%d",
                message.data.size(), message.data.empty() ? 0 : message.data[0], cycle);

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
