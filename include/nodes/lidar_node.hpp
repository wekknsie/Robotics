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
            state_->lidarFront = (double)data.back;

            //RCLCPP_INFO(this->get_logger(), "Data from lidar: \n front:'%f',back:'%f',left:'%f',right:'%f'",data.front,data.back,data.left,data.right);
      
        }

        LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI / 4.0f;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                // TODO: Skip invalid (infinite) readings
                if(std::isinf(points[i]) || std::isnan(points[i])) {
                    continue;
                }

                // TODO: Sort the value into the correct directional bin based on angle
                if(angle >= -angle_range && angle <= angle_range) {
                    front.push_back(points[i]);
                } else if(angle > angle_range && angle <= M_PI - angle_range) {
                    left.push_back(points[i]);
                } else if(angle < -angle_range && angle >= -M_PI + angle_range) {
                    right.push_back(points[i]);
                } else {
                    back.push_back(points[i]);
                }
                
            }

            // TODO: Return the average of each sector (basic mean filter)
            float sumFront = std::accumulate(front.begin(), front.end(), 0.0);
            float sumBack = std::accumulate(back.begin(), back.end(), 0.0);            
            float sumLeft = std::accumulate(left.begin(), left.end(), 0.0);
            float sumRight = std::accumulate(right.begin(), right.end(), 0.0);

            return LidarFilterResults{
                .front = sumFront / front.size(),
                .back = sumBack / back.size(),
                .left = sumLeft / left.size(),
                .right = sumRight / right.size(),
            };
        }
        
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::shared_ptr<SharedState> state_;
};
