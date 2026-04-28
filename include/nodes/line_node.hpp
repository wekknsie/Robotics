#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "shared_place.hpp"

// Calibration values for the line sensors
const uint16_t MIN_SENSOR_VALUE = 0;
const uint16_t MAX_SENSOR_VALUE = 930;



using std::placeholders::_1;

class LineNode : public rclcpp::Node
{
	public:
    LineNode(std::shared_ptr<SharedState> state)
    : Node("line_node"), state_(state)
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/bpc_prp_robot/line_sensors", 10, std::bind(&LineNode::topic_callback, this, _1));
    }

  private:
		void topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
			if (msg->data.size() < 2) {
				return;
			}

			double left_sensor = normalize(msg->data[0]);
			double right_sensor = normalize(msg->data[1]);

			state_->left_sensor.store(left_sensor);
			state_->right_sensor.store(right_sensor);

    //   double error=rror(left_sensor,right_sensor);

      //RCLCPP_INFO(this->get_logger(), "Left sensor: '%.3f' | Right sensor: '%.3f'",left_sensor,right_sensor);
    }

		double normalize(uint16_t data)
		{
			double norm_data = 1 - (2*(data/(double)MAX_SENSOR_VALUE));
			return norm_data;
		}
		
		// double Error(double l, double r)
		// {
		// 	double error = (l-r)/(l+r);
		// 	return error;
		// }

	rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
    std::shared_ptr<SharedState> state_;

};
