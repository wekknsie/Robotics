#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include "shared_place.hpp"
#include "io_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr float WHEEL_BASE = 0.12f;
constexpr float WHEEL_RADIUS = 0.033f;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 550;

class MotorNode : public rclcpp::Node
{
public:
  MotorNode(std::shared_ptr<SharedState> state)
  : Node("motor_node"), count_(0), state_(state)
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MotorNode::timer_callback, this));

    subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
      "/bpc_prp_robot/encoders",
      10,
      [this](std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        this->encoder_callback(msg);
      }
    );
  }

private:
  /**
   * @brief Publishing motors speed
   * 
   */
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt8MultiArray();
    
    message.data = {speedLeftWheel, speedRightWheel};

    publisher_->publish(message);
  }

  /**
   * @brief Processing encoder readings to determine wheel rotations and adjust speed accordingly
   * 
   * @param msg Input message containing encoder readings for both wheels
   */
  void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) { // if we don't have both left and right encoder readings, ignore the message
      return;
    }

    int button_state = state_->last_button.load();
    if(button_state == 0){
      speedLeftWheel = 127;
      speedRightWheel = 127;
      return;
    }

    uint32_t nowL = msg->data[0]; // store both current left and right encoder readings
    uint32_t nowR = msg->data[1];

    if(!havePrevious){ // if this is the first reading, just store the current readings
      prevLeft = nowL;
      prevRight = nowR;
      havePrevious = true;
      return;
    }

    int64_t deltaLeft = calculateTicks(nowL, prevLeft); // calculate the number of ticks since the last reading for both wheels
    int64_t deltaRight = calculateTicks(nowR, prevRight);
    prevLeft = nowL; // set previous readings to current for the next callback
    prevRight = nowR;

    accumulatedLeft += deltaLeft; // accumulate the ticks for both wheels to track total distance traveled
    accumulatedRight += deltaRight;

    if(std::llabs(accumulatedLeft) >= PULSES_PER_ROTATION){ // when wheel did a full rotation
      //RCLCPP_INFO(this->get_logger(), "Left wheel made a rotation, ticks error: %ld", accumulatedLeft - PULSES_PER_ROTATION);
      accumulatedLeft = 0;
    }
    if(std::llabs(accumulatedRight) >= PULSES_PER_ROTATION){
      //RCLCPP_INFO(this->get_logger(), "Right wheel made a rotation, ticks error: %ld", accumulatedRight + PULSES_PER_ROTATION);
      accumulatedRight = 0;
    }

    double dsL = (double)deltaLeft  * (WHEEL_CIRCUMFERENCE / (double)PULSES_PER_ROTATION);
    double dsR = std::abs((double)deltaRight * (WHEEL_CIRCUMFERENCE / (double)PULSES_PER_ROTATION));
    leftWheelDistance += dsL;
    rightWheelDistance += dsR;

    double d0 = (dsR - dsL) / (double)WHEEL_BASE;
    double dS = (dsR + dsL) / 2;
   
    x += dS * cos(theta + d0/2);
    y += dS * sin(theta + d0/2);
    
    theta += d0;
    
    //handleStraightMovement(theta);
    if(button_state == 2){
      regulatorPID_line(state_->left_sensor - state_->right_sensor);
    } else if(button_state == 1){
      corridorNavigation();
    }
    

    //RCLCPP_INFO(this->get_logger(), "Delta Left: %f m, Delta Right: %f m", dsL, dsR);
    //RCLCPP_INFO(this->get_logger(), "Total Left: %f m, Total Right: %f m", leftWheelDistance, std::abs(rightWheelDistance));
    /*RCLCPP_INFO(this->get_logger(),
            "dsL=%.4f m, dsR=%.4f m | pose: x=%.3f y=%.3f theta=%.3f, speedLeftWheel=%u, speedRightWheel=%u",
            dsL, dsR, x, y, theta, speedLeftWheel, speedRightWheel);
    */
    //RCLCPP_INFO(this->get_logger(), "Encoder[0]=%u", msg->data[0]);
  }

  /**
   * @brief Primitive function to handle straight movement
   * 
   * @param theta 
   */
  void handleStraightMovement(const double theta){
    if(std::abs(theta) < 0.1){
      speedLeftWheel = 140;
      speedRightWheel = 140;
    } else if(theta > 0){
      speedLeftWheel = 140;
      speedRightWheel = 130;
    } else {
      speedLeftWheel = 130;
      speedRightWheel = 140;
    }
  }

  void corridorNavigation(){
    switch(corridorState){
      case CALIBRATION:
        // TODO make calibration to gain data from IMU
        integral_ = 0.0;
        corridorState = CORRIDOR_NAVIGATION;
        break;
      case CORRIDOR_NAVIGATION:{
        double front = state_->lidarFront;
        double left = state_->lidarLeft;
        double right = state_->lidarRight;

        RCLCPP_INFO(this->get_logger(), "Lidar Front: %.4f, Left: %.4f, Right: %.4f", front, left, right);

        const double frontStop = 0.2;
        const double frontSlow = 0.3;

        if(front < frontStop){ // NEED TO TURN, stop and change state to TURNING
          integral_ = 0.0;
          speedLeftWheel = 127;
          speedRightWheel = 127;
          corridorState = TURNING;
          break;
        }
        
        int baseSpeedCorridor = 140;
        if(front < frontSlow){
          baseSpeedCorridor = 135; // TODO tune this value after testing
        }
             

        double error = right - left;
        regulatorPID_lidar(error, baseSpeedCorridor);
        
        break;
      }
      case TURNING:
        RCLCPP_INFO(this->get_logger(), "TURNING");
        break;
    }
  }

  void regulatorPID_lidar(double error, int baseSpeed){
    static auto last_time = this->now();

    auto now = this->now();
    double dt = (now - last_time).seconds();
    last_time = now;

    if (dt <= 0.0) dt = 0.01;
    if (dt > 0.05) dt = 0.05;
    
    //double dt = 0.01;

    double kp = 2.0;
    double kd = 0.08;
    double ki = 0.0;
    
    if (std::abs(error) < 0.05) {
        error = 0.0;
    }

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -10.0, 10.0);

    double derivative = (error - prev_error_) / dt;
    derivative = std::clamp(derivative, -1.0, 1.0);

    double correction = 5 * (kp * error + ki * integral_ + kd * derivative);
    
    int left = static_cast<int>(baseSpeed - correction);
    int right = static_cast<int>(baseSpeed + correction);

    left = std::clamp(left, 127, 155);
    right = std::clamp(right, 127, 155);

    speedLeftWheel = static_cast<uint8_t>(left);
    speedRightWheel = static_cast<uint8_t>(right);

    RCLCPP_INFO(this->get_logger(), "Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d", error, correction, left, right);

    prev_error_ = error;
  }

  void regulatorPID_line(const double error){
    //double error = (state_->left_sensor - state_->right_sensor); 
    
    // bad values
    //double dt = 0.05;
    //double k =  4;
    //double ki = 0.0;
    //double kd = 0.01;
    
    // FINAL VALUES
    double dt = 0.05;
    double k = 5.0;
    double ki = 0.0;
    double kd = 0.02;

    double Kp = k * error;
    integral_ += error *dt;
    double Ki =  integral_*ki;

    double derivative = (error - prev_error_) / dt;
    /*double Kd = derivative*kd;
    double correction = Kp + Ki+ Kd;*/

    double correction = k * error + ki * integral_ + kd * derivative;

    int baseSpeed = 135;
    
    if(std::abs(error) > 0.4) { // Need to turn more sharply, reduce base speed to allow for greater correction
      baseSpeed = 130;
    }

    int left = static_cast<int>(baseSpeed + correction);
    int right = static_cast<int>(baseSpeed - correction);

    left = std::clamp(left, 127, 150);
    right = std::clamp(right, 127, 150);

    RCLCPP_INFO(this->get_logger(), "Error: %.4f, Correction: %.4f, Left: %d, Right: %d", error, correction, left, right);

    speedLeftWheel = static_cast<uint8_t>(left);
    speedRightWheel = static_cast<uint8_t>(right);

    prev_error_ = error;
  }


  /**
   * @brief Calculate the number of ticks between two encoder readings, accounting for overflow
   * 
   * @param now Current encoder reading
   * @param prev Previous encoder reading
   * @return uint64_t ticks between the two readings, adjusted for overflow
   */
  static int64_t calculateTicks(uint32_t now, uint32_t prev){
    int64_t ticks = (int64_t)now - (int64_t)prev;

    if(ticks > (1LL<<31)){
      ticks -= (1LL<<32);
    }
    if(ticks < -(1LL<<31)){
      ticks += (1LL<<32);
    }

    return ticks;
  }

  // Corridor navigation variables
  enum corridor_state {
    CALIBRATION = 0,
    CORRIDOR_NAVIGATION = 1,
    TURNING = 2
  };

  corridor_state corridorState = CALIBRATION;

  // -----------------------------------
  
  double prev_error_ = 0.0;
  double integral_ = 0.0;

  uint32_t prevLeft = 0;
  int64_t accumulatedLeft = 0;
  uint32_t prevRight = 0;
  int64_t accumulatedRight = 0;

  double leftWheelDistance = 0.0;
  double rightWheelDistance = 0.0;

  uint8_t speedLeftWheel = 140;
  uint8_t speedRightWheel = 140;

  double x = 0;
  double y = 0;
  double theta = 0.0;

  bool havePrevious = false;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::shared_ptr<SharedState> state_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
};
