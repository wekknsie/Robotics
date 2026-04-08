#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "shared_place.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

class CameraNode : public rclcpp::Node
{
	public:
    CameraNode(std::shared_ptr<SharedState> state)
    : Node("camera_node"), state_(state)
    {
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

      subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "bpc_prp_robot/camera/compressed", 10, std::bind(&CameraNode::topic_callback, this, _1));
    }

    struct Aruco {
      int id;
      std::vector<cv::Point2f> corners;
    };

    ~CameraNode() = default;

    std::vector<Aruco> detect(cv::Mat frame) {
      std::vector<Aruco> arucos;

      std::vector<int> marker_ids;
      std::vector<std::vector<cv::Point2f>> marker_corners,rejectedCandidates;

      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, parameters, rejectedCandidates); 
      
      cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

      if (!marker_ids.empty()) {
          std::cout << "Arucos found: ";
          for (size_t i = 0; i < marker_ids.size(); i++) {
              std::cout << marker_ids[i] << " ";
              
              // TODO: Create Aruco struct and add to result vector
              // arucos.emplace_back(...);
            arucos.emplace_back(Aruco{marker_ids[i], marker_corners[i]});

          }
          std::cout << std::endl;
      }

    return arucos;
  }

  private:
		void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
      cv::Mat buffer(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));

      // dekódování do BGR obrazu
      cv::Mat image = cv::imdecode(buffer, cv::IMREAD_COLOR);

      if (image.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Nepodařilo se dekódovat compressed image.");
          return;
      }

      auto arucos = detect(image);

      //cv::imshow("image", image);
      //cv::waitKey(1);
    }

	cv::Ptr<cv::aruco::Dictionary> dictionary_;

	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  std::shared_ptr<SharedState> state_;

};