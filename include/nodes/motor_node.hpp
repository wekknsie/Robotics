#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <atomic>

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


constexpr uint8_t MOTOR_NEUTRAL = 127;
constexpr double TURN_STEP_RAD = M_PI / 2.0;
constexpr double MAX_YAW_ERROR_RAD = 13.0 * M_PI / 180.0;
constexpr int TURNING_MOTOR_DELTA = 10;

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
    
    message.data = {speedLeftWheel.load(), speedRightWheel.load()};

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
      stopMotors();
      resetPid();
      corridorState = CALIBRATION;
      havePrevious = false;
      counter = 0;
      return;
    }

    if(!state_->imuReady.load()){
      stopMotors();
      return;
    }
    
    //handleStraightMovement(theta);
    if(button_state == 2){
      regulatorPID_line(state_->left_sensor.load() - state_->right_sensor.load());
    } else if(button_state == 1){
      //testIMU();
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

  int stateTestIMU = 0;
  int motorSpeedDiff = 6;

  double stopEarlyAngle = 0.18; // try increase a little bit

  void testIMU()
  {
      auto now = this->now();
      double yaw = state_->imuAngle.load();

      if (first) {
          startYaw = yaw;
          start_time = now;
          first = false;
      }

      double turnAngle = std::abs(normalizeAngle(yaw - startYaw));

      RCLCPP_INFO(
          this->get_logger(),
          "Testing IMU, Yaw: %.4f, Start Yaw: %.4f, Turn angle: %.4f, State: %d",
          yaw, startYaw, turnAngle, stateTestIMU
      );

      constexpr double TARGET = M_PI / 2.0;

      auto getTurnDiff = [&](double remaining) {
          if (remaining > 0.70) {
              return motorSpeedDiff;      // rychle
          } else if (remaining > 0.35) {
              return 4;                   // středně
          } else {
              return 2;                   // pomalu před cílem
          }
      };

      if (stateTestIMU == 0) {
          double remaining = TARGET - turnAngle;

          if (remaining <= stopEarlyAngle) {
              stopMotors();
              start_time = now;
              stateTestIMU = 1;
              return;
          }

          int diff = getTurnDiff(remaining);
          setMotorSpeeds(127 + diff, 127 - diff);
      }
      else if (stateTestIMU == 1) {
          stopMotors();

          if ((now - start_time).seconds() > 1.0) {
              first = true;
              stateTestIMU = 2;
          }
      }
      else if (stateTestIMU == 2) {
          double remaining = TARGET - turnAngle;

          if (remaining <= stopEarlyAngle) {
              stopMotors();
              start_time = now;
              stateTestIMU = 3;
              return;
          }

          int diff = getTurnDiff(remaining);
          setMotorSpeeds(127 - diff, 127 + diff);
      }
      else if (stateTestIMU == 3) {
          stopMotors();

          if ((now - start_time).seconds() > 1.0) {
              first = true;
              stateTestIMU = 0;
          }
      }
  }
 
  void corridorNavigation(){
    double front = state_->lidarFront.load();
    double left = state_->lidarLeft.load();
    double right = state_->lidarRight.load();

    switch(corridorState){
      case CALIBRATION:{
        resetPid();
        stopMotors();
        counter = 0;
        corridorState = CORRIDOR_NAVIGATION;
        break;
      }
      case CORRIDOR_NAVIGATION:{
        RCLCPP_INFO(this->get_logger(), "Lidar Front: %.4f, Left: %.4f, Right: %.4f", front, left, right);

        const double frontStop = 0.27;
        int baseSpeedCorridor = 140;
        const double frontSlow = 0.1;

        if(std::isfinite(front) && front < frontStop){
          resetPid();
          stopMotors();

          startYaw = state_->imuAngle.load();
          turnDirection_ = chooseTurnDirection(left, right);
    //      targetYaw = normalizeAngle(roundToRightAngle(startYaw + turnDirection_ * TURN_STEP_RAD));
          counter = 0;
          corridorState = TURNING;

          break;
        }

        if(std::isfinite(front) && front < frontSlow){
          resetPid();
          setMotorSpeeds(132, 132); 

          break;
        }
             

        constexpr double sideMaxDistance = 0.45;
        double leftForControl = cappedDistance(left, sideMaxDistance);
        double rightForControl = cappedDistance(right, sideMaxDistance);
        double error = leftForControl - rightForControl;
        if(leftForControl > 0.35 || rightForControl > 0.35){
            // if(!(leftForControl > 0.80 && rightForControl > 0.80)){
              error = 0.0;
            //} 
        }
        
        RCLCPP_INFO(this->get_logger(), "Corridor Error: %.4f Left %.4f Right %.4f", error, left, right);
        regulatorPID_lidar(error, baseSpeedCorridor);
        
        break;
      }
      case TURNING:{ // TODO: create turning function
        
        double yaw = state_->imuAngle.load();
        double turnAngle = std::abs(normalizeAngle(yaw - startYaw));
        // double yawError = angleDifference(targetYaw, yaw);

        constexpr double TARGET = M_PI / 2.0;

        auto getTurnDiff = [&](double remaining) {
            if (remaining > 0.70) {
                return motorSpeedDiff;      // rychle
            } else if (remaining > 0.35) {
                return 4;                   // středně
            } else {
                return 2;                   // pomalu před cílem
            }
        };
        
        double remaining = TARGET - turnAngle;

          if (remaining <= stopEarlyAngle) {
              stopMotors();
              corridorState = END;
              break;
          }

        int diff = getTurnDiff(remaining);

        //  if(std::abs(yawError) <= MAX_YAW_ERROR_RAD){
        //     corridorState = END;
        //     stopMotors();
        //     break;
        //   }

        //   corridorTurning(sign(yawError));
        //   RCLCPP_INFO(this->get_logger(),
        //     "TURNING, Yaw: %.4f, Target Yaw: %.4f, Error: %.4f, Direction: %d",
        //     yaw, targetYaw, yawError, sign(yawError));

        corridorTurning(turnDirection_, diff);
        // RCLCPP_INFO(this->get_logger(), "TURNING, Yaw: %.4f, Start Yaw: %.4f, Turn angle: %.4f, Direction: %d", yaw, startYaw, turnAngle, turnDirection_);
        break;
      }
      case END:{
        setMotorSpeeds(134, 134);
        if(++counter > 75){
          counter = 0;
          resetPid();
          corridorState = CORRIDOR_NAVIGATION;
        }

        RCLCPP_INFO(this->get_logger(), "POPOJIZDIM - %d", counter);
        break;
      }
    }
  }

  void regulatorPID_lidar(double error, int baseSpeed){
    static auto last_time = this->now();

    auto now = this->now();
    double dt = (now - last_time).seconds();
    last_time = now;

    if (dt <= 0.0) dt = 0.01;
    if (dt > 0.05) dt = 0.05;
    
    double kp = 5.0;
    double kd = 2.0;
    double ki = 0.0;
    
    if (std::abs(error) < 0.025) {
        error = 0.0;
    }

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -0.5, 0.5);

    double derivative = (error - prev_error_) / dt;
    derivative = std::clamp(derivative, -1.0, 1.0);

    double correction = 1.8 * (kp * error + kd * derivative + ki * integral_);
    
    int left = static_cast<int>(baseSpeed - correction);
    int right = static_cast<int>(baseSpeed + correction);

    left = std::clamp(left, 127, 155);
    right = std::clamp(right, 127, 155);

    setMotorSpeeds(left, right);

    if(left > right){
      RCLCPP_INFO(this->get_logger(), "Turning RIGHT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
    } else if(right > left){
      RCLCPP_INFO(this->get_logger(), "Turning LEFT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
    } else {
      RCLCPP_INFO(this->get_logger(), "Moving STRAIGHT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
    }

    prev_error_ = error;
  }

  void corridorTurning(int direction, int diff){
    if(direction > 0){ // turn left
      setMotorSpeeds(127+diff, 127-diff);
    } else { // turn right
      setMotorSpeeds(127-diff, 127+diff);
    }


  }

  void regulatorPID_line(const double error){
    //double error = (state_->left_sensor - state_->right_sensor); 
    
    // bad values
    //double dt = 0.05;
    //double k =  4;
    //double ki = 0.0;
    //double kd = 0.01;
    
    // FINAL VALUES
    double dt = 0.01;
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

    left = std::clamp(left, 127, 140);
    right = std::clamp(right, 127, 140);

    RCLCPP_INFO(this->get_logger(), "Error: %.4f, Correction: %.4f, Left: %d, Right: %d", error, correction, left, right);

    setMotorSpeeds(left, right);

    prev_error_ = error;
  }

  void setMotorSpeeds(int left, int right)
  {
    left = std::clamp(left, 0, 255);
    right = std::clamp(right, 0, 255);
    speedLeftWheel.store(static_cast<uint8_t>(left));
    speedRightWheel.store(static_cast<uint8_t>(right));
  }

  void stopMotors()
  {
    setMotorSpeeds(127, 127);
  }

  void resetPid()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

  static double cappedDistance(double distance, double maxDistance)
  {
    if (!std::isfinite(distance)) {
      return maxDistance;
    }
    return std::clamp(distance, 0.0, maxDistance);
  }

  static double clearanceForTurn(double distance)
  {
    if (std::isnan(distance)) {
      return -1.0;
    }
    if (std::isinf(distance)) {
      return 10.0;
    }
    return distance;
  }

  int chooseTurnDirection(double left, double right) const
  {
    constexpr double sameClearanceEpsilon = 0.05;
    double leftClearance = clearanceForTurn(left);
    double rightClearance = clearanceForTurn(right);

    if (std::abs(leftClearance - rightClearance) < sameClearanceEpsilon) {
      RCLCPP_INFO(this->get_logger(), "Both sides have similar clearance (Left: %.4f, Right: %.4f), defaulting to right turn", leftClearance, rightClearance);
      return -1; // Default to a right turn if both sides look equally open.
    }
    RCLCPP_INFO(this->get_logger(), "Choosing turn direction based on clearance (Left: %.4f, Right: %.4f)", leftClearance, rightClearance);
    return leftClearance > rightClearance ? -1 : 1;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }
    // static double angleDifference(double target, double current)
    // {
    //   return normalizeAngle(target - current);
    // }

    // static double roundToRightAngle(double angle)
    // {
    //   return std::round(angle / TURN_STEP_RAD) * TURN_STEP_RAD;
    // }

    // static int sign(double value)
    // {
    //   return (0.0 < value) - (value < 0.0);
    // }

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
    TURNING = 2,
    END = 3
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

  std::atomic<uint8_t> speedLeftWheel{127};
  std::atomic<uint8_t> speedRightWheel{127};

  double x = 0;
  double y = 0;
  double theta = 0.0;

  double startYaw = 0.0;
  double targetYaw = 0.0;
  int turnDirection_ = -1;
  bool first = true;
  rclcpp::Time last_time;
  rclcpp::Time start_time;
  int counter = 0;

  bool havePrevious = false;
  rclcpp::TimerBase::SharedPtr timer_;
  
  size_t count_;
  std::shared_ptr<SharedState> state_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
};
