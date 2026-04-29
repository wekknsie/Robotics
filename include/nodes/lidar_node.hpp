#pragma once
#include <cmath>
#include <algorithm>
#include <limits>
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
            LidarFilterResults data = apply_filter(*msg);

            state_->lidarLeft.store((double)data.left);
            state_->lidarRight.store((double)data.right);
            state_->lidarFront.store((double)data.front);

            //RCLCPP_INFO(this->get_logger(), "Data from lidar: \n front:'%f',back:'%f',left:'%f',right:'%f'",data.front,data.back,data.right,data.left);
      
        }

        LidarFilterResults apply_filter(const sensor_msgs::msg::LaserScan& scan) {
            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};
            const auto& points = scan.ranges;

            if (points.empty()) {
                const float inf = std::numeric_limits<float>::infinity();
                return LidarFilterResults{inf, inf, inf, inf};
            }

            constexpr float front_window = M_PI / 9.0f;
            constexpr float side_window = M_PI / 10.0f;
            constexpr float lidar_to_robot_yaw = M_PI;

            // Compute the angular step between each range reading
            const float angle_step = scan.angle_increment != 0.0f
                ? scan.angle_increment
                : (points.size() > 1 ? (scan.angle_max - scan.angle_min) / (points.size() - 1) : 0.0f);

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = normalizeAngle(scan.angle_min + i * angle_step + lidar_to_robot_yaw);
                // const float range = points[i];
                 float range = normalizeRange(points[i], scan.range_min, scan.range_max);
                // TODO: Skip invalid (infinite) readings
                if(!std::isfinite(range)) {
                    continue;
                }

                if(std::abs(angle) <= front_window) {
                    front.push_back(range);
                } else if(std::abs(angle - M_PI / 2.0f) <= side_window) {
                    left.push_back(range);
                } else if(std::abs(angle + M_PI / 2.0f) <= side_window) {
                    right.push_back(range);
                } else if(std::abs(std::abs(angle) - M_PI) <= front_window) {
                    back.push_back(range);
                }
                
            }

            // TODO: Return the average of each sector (basic mean filter)
            return LidarFilterResults{
                median(front),
                median(back),
                median(left),
                median(right),
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
   float normalizeRange(float range, float range_min, float range_max) {
            if (std::isnan(range)) {
                return std::numeric_limits<float>::infinity();
            }

            if (std::isinf(range)) {
                return range < 0.0f ? range_min : std::numeric_limits<float>::infinity();
            }

            if (range < range_min) {
                return range_min;
            }

            if (range > range_max) {
                return std::numeric_limits<float>::infinity();
            }

            return range;
        }

        float normalizeAngle(float angle) {
            while (angle > M_PI) {
                angle -= 2.0f * M_PI;
            }
            while (angle < -M_PI) {
                angle += 2.0f * M_PI;
            }
            return angle;
        }
        
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::shared_ptr<SharedState> state_;
};