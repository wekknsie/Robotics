//////////////////////////////////////////////
// filename: imu_node.hpp                   //
// Project Blueberry                      	//
// Authors:						  			//
//  * Jan Hájek / wekknsie               	//
//  * Horbal Volodymyr-Mykhailo / Mr-Vova   //
//////////////////////////////////////////////

#pragma once
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "shared_place.hpp"

using std::placeholders::_1;

class ImuNode : public rclcpp::Node
{
public:
    ImuNode(std::shared_ptr<SharedState> state)
        : Node("imu_node"), state_(state)
    {
        state_->imuReady.store(false);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 50, std::bind(&ImuNode::topic_callback, this, _1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double gyro_z = msg->angular_velocity.z;
        if (!std::isfinite(gyro_z))
        {
            return;
        }

        rclcpp::Time now = msg->header.stamp;
        if (now.nanoseconds() == 0)
        {
            now = this->now();
        }

        if (first)
        {
            start_time = now;
            last_time = now;
            first = false;
            return;
        }

        double dt = (now - last_time).seconds();
        last_time = now;

        if (dt <= 0.0 || dt > 0.5)
        {
            return;
        }

        if (!calibrated)
        {
            samples.push_back(gyro_z);

            double elapsed = (now - start_time).seconds();
            if (elapsed > 5.0) // Five seconds to gather calibration data
            {
                if (samples.empty())
                {
                    return;
                }

                double sum = 0.0;
                for (double s : samples)
                {
                    sum += s;
                }

                offset = sum / static_cast<double>(samples.size());
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

        // RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);
    }

    double yaw = 0.0;
    double offset = 0.0;
    rclcpp::Time last_time;
    std::vector<double> samples;
    bool calibrated = false;
    rclcpp::Time start_time;
    bool first = true;

    std::shared_ptr<SharedState> state_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};