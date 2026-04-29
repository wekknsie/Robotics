#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "shared_place.hpp"


using std::placeholders::_1;

class ImuNode : public rclcpp::Node
{
	public:
        ImuNode(std::shared_ptr<SharedState> state)
        : Node("imu_node"), state_(state)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10, std::bind(&ImuNode::topic_callback, this, _1));
        }

    private:

        void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            float gyro_z = msg->angular_velocity.z;
            rclcpp::Time now = this->now();

            if(first){
                start_time = now;
                last_time = now;
                first = false;
                return;
            }

            double dt = (now - last_time).seconds();
            last_time = now;

            if (dt <= 0.0 || dt > 0.1) {
                return;
            }

            if (!calibrated) {
                samples.push_back(gyro_z);

                double elapsed = (now - start_time).seconds();
                if (elapsed > 5.0) {
                    if (samples.empty()) {
                        return;
                    }

                    float sum = 0.0f;
                    for (float s : samples) {
                        sum += s;
                    }

                    offset = sum / samples.size();
                    yaw = 0.0f;

                    state_->imuAngle.store(0.0);
                    state_->imuReady.store(true);

                    RCLCPP_INFO(this->get_logger(), "IMU Calibrated with offset: %f", offset);
                    calibrated = true;
                }

                return;
            }

            yaw += (gyro_z - offset) * dt;
            yaw = std::atan2(std::sin(yaw), std::cos(yaw));
            state_->imuAngle.store(yaw);

            //RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);
        }

        float yaw = 0.0;
        rclcpp::Time last_time;
        std::vector<float> samples;
        bool calibrated = false;
        float offset = 0.0;
        rclcpp::Time start_time;
        bool first = true;

        std::shared_ptr<SharedState> state_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};