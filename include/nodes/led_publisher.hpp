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

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(std::shared_ptr<SharedState> state)
  : Node("minimal_publisher"), count_(0), state_(state)
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::UInt8MultiArray message;

    int button_state = state_->last_button.load();
    RCLCPP_INFO(this->get_logger(), "Button state: %d", button_state);

    if(button_state == 0)
    {
        //message.layout.data_offset = 1;
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
        cycle = (cycle + 1) % 8;
        
        switch(cycle){
            case 0:{
                for(int i = 0; i < 150; i += 15){                    
                    message.data = {
                        i,i,i, 0,0,0, 0,0,0, 0,0,0, 
                    };

                    publisher_->publish(message);
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                }

                message.data = {
                        0,0,0, 0,0,0, 0,0,0, 0,0,0, 
                    };
                publisher_->publish(message);

                return;
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
  int cycle = 0;
  std::shared_ptr<SharedState> state_;
};
