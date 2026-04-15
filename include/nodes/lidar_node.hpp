#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shared_place.hpp"


using std::placeholders::_1;

class LidarNode : public rclcpp::Node
{
	public:
        LidarNode(std::shared_ptr<SharedState> state)
        : Node("lidar_node"), state_(state)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10, std::bind(&LidarNode::topic_callback, this, _1));
        }

    private:
        struct LidarFilterResults {
            float front;
            float back;
            float left;
            float right;    
        };
        
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            LidarFilterResults data;
            data = apply_filter(msg->ranges, msg->angle_min, msg->angle_max);

            state_->lidarLeft = (double)data.left;
            state_->lidarRight = (double)data.right;
            state_->lidarFront = (double)data.front;
      
        }

        LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // Define how wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI / 4.0f;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / static_cast<float>(points.size() - 1);

            for (size_t i = 0; i < points.size(); ++i) {
                float angle = angle_start + static_cast<float>(i) * angle_step;

                // Skiping invalid (infinite) readings
                if(std::isinf(points[i]) || std::isnan(points[i])) {
                    continue;
                }

                // Sort the value into the correct directional bin based on angle
                if(angle >= deg2rad(165.0f) || angle <= deg2rad(-165.0f)) {
                    front.push_back(points[i]);
                } else if(angle >= deg2rad(75.0f) && angle <= deg2rad(105.0f)) {
                    right.push_back(points[i]);
                } else if(angle >= deg2rad(-105.f) && angle <= deg2rad(-75.0f)) {
                    left.push_back(points[i]);
                } else {
                    back.push_back(points[i]);
                }
                
            }

            //RCLCPP_INFO(this->get_logger(), "LIDAR filter results: \n front:'%f',back:'%f',left:'%f',right:'%f'",median(front),median(back),median(left),median(right));

            return LidarFilterResults{
                .front = median(front),
                .back  = median(back),
                .left  = median(left),
                .right = median(right),
            };
        }

        float median(std::vector<float> values) {
            if (values.empty()) return std::numeric_limits<float>::infinity();

            std::sort(values.begin(), values.end());
            size_t n = values.size();

            if (n % 2 == 0) {
                return (values[n / 2 - 1] + values[n / 2]) / 2.0f;
            }
            return values[n / 2];
        }

        float deg2rad(float degrees) {
            return degrees * M_PI / 180.0f;
        }
        
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::shared_ptr<SharedState> state_;
};
