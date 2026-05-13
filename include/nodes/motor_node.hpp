#pragma once

#include <functional>
#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "shared_place.hpp"

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
        : Node("motor_node"), state_(state)
    {
        stopMotors();
        resetPid();

        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 20);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&MotorNode::timer_callback, this));

        subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/bpc_prp_robot/encoders",
            20,
            [this](std_msgs::msg::UInt32MultiArray::SharedPtr msg)
            {
                this->encoder_callback(msg);
            });
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
        if (msg->data.size() < 2)
        { // if we don't have both left and right encoder readings, ignore the message
            return;
        }

        if (!state_->imuReady.load())
        { // wait till IMU is ready
            stopMotors();
            return;
        }

        int button_state = state_->last_button.load();
        if (button_state == 0)
        {
            stopMotors();
            resetPid();
            state_->corridorState.store(CorridorState::IDLE);
            corridorState = CALIBRATION;
            counter = 0;
            return;
        }

        if (button_state == 2)
        { // maze escape (old line following)
          // regulatorPID_line(state_->left_sensor.load() - state_->right_sensor.load());
        }
        else if (button_state == 1)
        { // corridor navigation
            //testIMU();
            corridorNavigation();
        }
    }

    int stateTestIMU = 0;
    int motorSpeedDiff = 6;
    double stopEarlyAngle = 0.18; // try increase a little bit

    void testIMU()
    {
        auto now = this->now();
        double yaw = state_->imuAngle.load();

        if (first)
        {
            startYaw = yaw;
            start_time = now;
            first = false;
        }

        double turnAngle = std::abs(normalizeAngle(yaw - startYaw));

        RCLCPP_INFO(
            this->get_logger(),
            "Testing IMU, Yaw: %.4f, Start Yaw: %.4f, Turn angle: %.4f, State: %d",
            yaw, startYaw, turnAngle, stateTestIMU);

        constexpr double TARGET = M_PI / 2.0;

        auto getTurnDiff = [&](double remaining)
        {
            if (remaining > 0.70)
            {
                return motorSpeedDiff; // fast
            }
            else if (remaining > 0.35)
            {
                return 4; // medium
            }
            else
            {
                return 2; // slow near target
            }
        };

        if (stateTestIMU == 0)
        {
            double remaining = TARGET - turnAngle;

            if (remaining <= stopEarlyAngle)
            {
                stopMotors();
                start_time = now;
                stateTestIMU = 1;
                return;
            }

            int diff = getTurnDiff(remaining);
            setMotorSpeeds(127 + diff, 127 - diff);
        }
        else if (stateTestIMU == 1)
        {
            stopMotors();

            if ((now - start_time).seconds() > 1.0)
            {
                first = true;
                stateTestIMU = 2;
            }
        }
        else if (stateTestIMU == 2)
        {
            double remaining = TARGET - turnAngle;

            if (remaining <= stopEarlyAngle)
            {
                stopMotors();
                start_time = now;
                stateTestIMU = 3;
                return;
            }

            int diff = getTurnDiff(remaining);
            setMotorSpeeds(127 - diff, 127 + diff);
        }
        else if (stateTestIMU == 3)
        {
            stopMotors();

            if ((now - start_time).seconds() > 1.0)
            {
                first = true;
                stateTestIMU = 0;
            }
        }
    }

    void corridorNavigation()
    {
        double front = state_->lidarFront.load();
        double left = state_->lidarLeft.load();
        double right = state_->lidarRight.load();
        switch (corridorState)
        {
        case CALIBRATION:
        {
            resetPid();
            stopMotors();
            counter = 0;

            baseYaw_ = state_->imuAngle.load();
            headingIndex_ = 0;
            targetYaw_ = baseYaw_;
            baseYawInitialized_ = true;

            state_->corridorState.store(CorridorState::IDLE);
            corridorState = CORRIDOR_NAVIGATION;
            break;
        }

        case CORRIDOR_NAVIGATION:
        {
            state_->corridorState.store(CorridorState::USING_LIDAR);

            double leftBeam = state_->lidarLeftBeam.load();
            double rightBeam = state_->lidarRightBeam.load();

            const double wallSeenDistance = 0.35;

            //bool wallSeenAfterTurn = (std::isfinite(left) && left < wallSeenDistance) ||
            //                    (std::isfinite(right) && right < wallSeenDistance);
            
            if(!leftWallSeen_){
                leftWallSeen_ = std::isfinite(left) && leftBeam < wallSeenDistance;
            }
            
            if(!rightWallSeen_){
                rightWallSeen_ = std::isfinite(right) && rightBeam < wallSeenDistance;
            }

            //RCLCPP_INFO(this->get_logger(), "RIGHT WALL SEEN? %d d: %f, LEFT WALL SEEN? %d d: %f", rightWallSeen_, rightBeam, leftWallSeen_, leftBeam);

            const double frontStop = 0.3;
            const double openSide = 0.50;
            const double openFront = 0.45;
            int baseSpeedCorridor = 145;

            bool frontOpen = !std::isfinite(front) || front > openFront;
            bool leftOpen  = !std::isfinite(left)  || left  > openSide;
            bool rightOpen = !std::isfinite(right) || right > openSide;
            bool intersectionDetected = (leftOpen && rightOpen) || (frontOpen && (leftOpen || rightOpen));

            if(intersectionFlag_){
                if(leftWallSeen_ && rightWallSeen_){
                    intersectionFlag_ = false;
                    intersectionCounter_ = 0;
                    std::cout << "Wall seen after turn, resetting intersection flag" << std::endl;
                }
            }else{
                if(intersectionDetected){
                    intersectionCounter_++;
                }else{
                    intersectionCounter_ = 0;
                }

                if(intersectionCounter_ > 10){
                    intersectionFlag_ = true;
                    std::cout << "Intersection detected, setting flag" << std::endl;
                    int aruco[2];
                    aruco[0] = state_->cameraData[0].load();
                    aruco[1] = state_->cameraData[1].load();
                    state_->cameraData[0].store(-1);
                    state_->cameraData[1].store(-1);
                    int direction = 0;
                    for(int i = 0; i < 2; i++){
                        std::cout << "ARUCO[" << i << "] = " << aruco[i] << std::endl;
                        switch(aruco[i]){
                        case 0:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: STRAIGHT");
                            direction = 0;
                            break;
                        case 1:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: LEFT");
                            direction = -1;
                            break;
                        case 2:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: RIGHT");
                            direction = 1;
                            break;
                        case 10:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: TREASURE FRONT");
                            break;
                        case 11:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: TREASURE LEFT");
                            break;
                        case 12:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: TREASURE RIGHT");
                            break;
                        case -99:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: NOTHING");
                            break;
                        case -1:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: NOTHING");
                            break;
                        default:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: UNKNOWN");
                            if(rightOpen){
                                direction = 1;
                            }else if(frontOpen){
                                direction = 0;
                            }else if(leftOpen){
                                direction = -1;
                            }
                        }
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Chosen direction: %d", direction);
                    rightWallSeen_ = false;
                    leftWallSeen_ = false;

                    if(direction != 0){
                        turnDirection_ = direction;
                        startYaw = state_->imuAngle.load();
                        corridorState = TURNING;
                        intersectionCounter_ = 0;
                        return;
                    }
                    
                    RCLCPP_INFO(
                        this->get_logger(),
                        "INTERSECTION detected: frontOpen=%d leftOpen=%d rightOpen=%d",
                        frontOpen, leftOpen, rightOpen
                    );
                    std::cout << "INTERSECTION detected: frontOpen=" << frontOpen << " leftOpen=" << leftOpen << " rightOpen=" << rightOpen << std::endl;
                }
            }

            /*if(intersectionIgnoreCounter_ > 0){
                intersectionIgnoreCounter_--;
                intersectionCounter_ = 0;
            }else{
                if(intersectionDetected){
                    intersectionCounter_++;
                    std::cout << "Intersection counter: " << intersectionCounter_ << std::endl;
                }else{
                    intersectionCounter_ = 0;
                }

                if(intersectionCounter_ > 5){
                    intersectionCounter_ = 0;

                    int aruco[2];
                    aruco[0] = state_->cameraData[0].load();
                    aruco[1] = state_->cameraData[1].load();
                    state_->cameraData[0].store(-1);
                    state_->cameraData[1].store(-1);
                    int direction = 0;
                    for(int i = 0; i < 2; i++){
                        std::cout << "ARUCO[" << i << "] = " << aruco[i] << std::endl;
                        switch(aruco[i]){
                        case 0:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: LEFT");
                            direction = 1;
                            break;
                        case 1:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: FRONT");
                            direction = 0;
                            break;
                        case 2:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: RIGHT");
                            direction = -1;
                            break;
                        default:
                            RCLCPP_INFO(this->get_logger(), "ARUCO detected: UNKNOWN");
                        }
                    }
                    

                    if(direction != 0){
                        turnDirection_ = direction;
                        startYaw = state_->imuAngle.load();
                        corridorState = TURNING;
                        intersectionCounter_ = 0;
                        return;
                    }
                    
                    RCLCPP_INFO(
                        this->get_logger(),
                        "INTERSECTION detected: frontOpen=%d leftOpen=%d rightOpen=%d",
                        frontOpen, leftOpen, rightOpen
                    );
                    std::cout << "INTERSECTION detected: frontOpen=" << frontOpen << " leftOpen=" << leftOpen << " rightOpen=" << rightOpen << std::endl;
                }
            }*/

            if (std::isfinite(front) && front < frontStop)
            {
                resetPid();
                stopMotors();

                turnDirection_ = chooseTurnDirection(left, right);
                startYaw = state_->imuAngle.load();

                counter = 0;
                corridorState = TURNING;

                break;
            }

            regulatorCorridorImuLidar(left, right, baseSpeedCorridor);
            break;
        }

        case TURNING:
        {
            RCLCPP_INFO(this->get_logger(), "TURNING, Front: %.3f, Left: %.3f, Right: %.3f", front, left, right);
            state_->corridorState.store(CorridorState::USING_IMU);

            double yaw = state_->imuAngle.load();
            double turnAngle = std::abs(normalizeAngle(yaw - startYaw));

            constexpr double TARGET = M_PI / 2.0;

            auto getTurnDiff = [&](double remaining)
            {
                if (remaining > 0.70)
                {
                    return motorSpeedDiff; // fast
                }
                else if (remaining > 0.35)
                {
                    return 4; // medium
                }
                else
                {
                    return 2; // slow near target
                }
            };

            double remaining = TARGET - turnAngle;

            if (remaining <= stopEarlyAngle)
            {
                stopMotors();
                resetPid();
                counter = 0;
                leftWallSeen_ = false;
                rightWallSeen_ = false;
                corridorState = AFTER_TURN_CRAWLING;
                break;
            }

            int diff = getTurnDiff(remaining);
            corridorTurning(turnDirection_, diff);
            // RCLCPP_INFO(this->get_logger(), "TURNING, Yaw: %.4f, Start Yaw: %.4f, Turn angle: %.4f, Direction: %d", yaw, startYaw, turnAngle, turnDirection_);
            break;
        }
        case AFTER_TURN_CRAWLING:
        {
            //stopMotors();
            setMotorSpeeds(130, 130);

            if (++counter > 25)
            {
                targetYaw_ = state_->imuAngle.load();

                resetPid();
                counter = 0;

                corridorState = CORRIDOR_NAVIGATION;
                intersectionIgnoreCounter_ = 50;
                intersectionFlag_ = true;
                state_->corridorState.store(CorridorState::USING_LIDAR);
            }

            break;
        }
        case END:
        {
            /*targetYaw_ = state_->imuAngle.load();

            resetPid();
            counter = 0;

            corridorState = CORRIDOR_NAVIGATION;
            state_->corridorState.store(CorridorState::USING_LIDAR);*/
            state_->corridorState.store(CorridorState::IDLE);
            stopMotors();
            break;
        }
        }
    }

    void regulatorPID_lidar(double error, int baseSpeed)
    {
        static auto last_time = this->now();

        auto now = this->now();
        double dt = (now - last_time).seconds();
        last_time = now;

        if (dt <= 0.0)
        {
            dt = 0.01;
        }
        if (dt > 0.05)
        {
            dt = 0.05;
        }

        constexpr double kp = 8.0;
        constexpr double kd = 1.8;
        constexpr double ki = 0.0;
    

        if (std::abs(error) < 0.012)
        {
            error = 0.0;
        }

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -0.5, 0.5);

        double derivative = (error - prev_error_) / dt;
        derivative = std::clamp(derivative, -1.0, 1.0);

        double correction = kp * error + kd * derivative + ki * integral_;
        correction = std::clamp(correction, -20.0, 20.0);

        int left = static_cast<int>(baseSpeed - correction);
        int right = static_cast<int>(baseSpeed + correction);

        left = std::clamp(left, 127, 155);
        right = std::clamp(right, 127, 155);

        setMotorSpeeds(left, right);
        prev_error_ = error;

        if (left > right)
        {
            RCLCPP_INFO(this->get_logger(), "Turning RIGHT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
        }
        else if (right > left)
        {
            RCLCPP_INFO(this->get_logger(), "Turning LEFT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Moving STRAIGHT, Lidar Error: %.4f, Correction: %.4f, Left: %d, Right: %d\n", error, correction, left, right);
        }
    }

    void regulatorCorridorImuLidar(double leftDist, double rightDist, int baseSpeed)
    {
        double yaw = state_->imuAngle.load();

        double headingError = normalizeAngle(targetYaw_ - yaw);

        constexpr double sideMaxDistance = 0.45;
        double left = cappedDistance(leftDist, sideMaxDistance);
        double right = cappedDistance(rightDist, sideMaxDistance);

        double wallError = left - right;

        if (left >= sideMaxDistance || right >= sideMaxDistance)
        {
            wallError = 0.0;
        }

        constexpr double kpHeading = 18.0;
        constexpr double kpWall = 25.0;

        //double correction = kpHeading * headingError + kpWall * wallError;
        //correction = std::clamp(correction, -15.0, 15.0);

        double headingCorrection = std::clamp(kpHeading * headingError, -4.0, 4.0);

        double wallCorrection = kpWall * wallError;

        double correction = headingCorrection + wallCorrection;

        int leftSpeed = static_cast<int>(std::lround(baseSpeed - correction));
        int rightSpeed = static_cast<int>(std::lround(baseSpeed + correction));

        leftSpeed = std::clamp(leftSpeed, 127, 155);
        rightSpeed = std::clamp(rightSpeed, 127, 155);

        setMotorSpeeds(leftSpeed, rightSpeed);

        RCLCPP_INFO(
            this->get_logger(),
            "CTRL headingErr=%.3f wallErr=%.3f corr=%.2f L=%d R=%d",
            headingError, wallError, correction, leftSpeed, rightSpeed
        );
    }

    /* // chatgpt version, not working well
    void regulatorCorridorImuLidar(double leftDist, double rightDist, int baseSpeed)
    {
        double yaw = state_->imuAngle.load();

        double headingError =
            normalizeAngle(targetYaw_ - yaw);

        constexpr double sideMaxDistance = 0.45;
        constexpr double desiredWallDistance = 0.22;

        double left = cappedDistance(leftDist, sideMaxDistance);
        double right = cappedDistance(rightDist, sideMaxDistance);

        bool leftValid = left < sideMaxDistance;
        bool rightValid = right < sideMaxDistance;

        double wallError = 0.0;

        if (leftValid && rightValid)
        {
            wallError = left - right;
        }
        else if (leftValid)
        {
            wallError = desiredWallDistance - left;
        }
        else if (rightValid)
        {
            wallError = right - desiredWallDistance;
        }

        constexpr double kpHeading = 3.0;
        constexpr double kpWall = 25.0;

        double headingCorrection =
            std::clamp(kpHeading * headingError, -4.0, 4.0);

        double wallCorrection =
            kpWall * wallError;

        double correction =
            headingCorrection + wallCorrection;

        correction =
            std::clamp(correction, -15.0, 15.0);

        int leftSpeed =
            static_cast<int>(std::lround(baseSpeed - correction));

        int rightSpeed =
            static_cast<int>(std::lround(baseSpeed + correction));

        leftSpeed = std::clamp(leftSpeed, 127, 155);
        rightSpeed = std::clamp(rightSpeed, 127, 155);

        setMotorSpeeds(leftSpeed, rightSpeed);

        RCLCPP_INFO(
            this->get_logger(),
            "CTRL headingErr=%.3f wallErr=%.3f "
            "headCorr=%.2f wallCorr=%.2f "
            "final=%.2f L=%d R=%d",
            headingError,
            wallError,
            headingCorrection,
            wallCorrection,
            correction,
            leftSpeed,
            rightSpeed
        );

        if (leftValid && rightValid)
        {
            if (std::abs(wallError) < 0.03)
            {
                targetYaw_ =
                    0.98 * targetYaw_ +
                    0.02 * yaw;
            }
        }
    }*/

    void corridorTurning(int direction, int diff)
    {
        if (direction > 0) 
        {
            // turn right
            setMotorSpeeds(127 + diff, 127 - diff);
        }
        else
        {
            // turn left
            setMotorSpeeds(127 - diff, 127 + diff);
        }
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
        if (!std::isfinite(distance))
        {
            return maxDistance;
        }
        return std::clamp(distance, 0.0, maxDistance);
    }

    static double clearanceForTurn(double distance)
    {
        if (std::isnan(distance))
        {
            return -1.0;
        }
        if (std::isinf(distance))
        {
            return 10.0;
        }
        return distance;
    }

    int chooseTurnDirection(double left, double right) const
    {
        constexpr double sameClearanceEpsilon = 0.05;

        double leftClearance = clearanceForTurn(left);
        double rightClearance = clearanceForTurn(right);

        if (std::abs(leftClearance - rightClearance) < sameClearanceEpsilon)
        {
            return 1; // default right
        }

        return leftClearance > rightClearance ? -1 : 1;
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    /*
    static int64_t calculateTicks(uint32_t now, uint32_t prev)
    {
        int64_t ticks = (int64_t)now - (int64_t)prev;

        if (ticks > (1LL << 31))
        {
            ticks -= (1LL << 32);
        }
        if (ticks < -(1LL << 31))
        {
            ticks += (1LL << 32);
        }

        return ticks;
    }
    */

    // -----------------------------------------------------
    // -----------------------------------------------------

    // Corridor navigation variables
    enum corridor_state
    {
        CALIBRATION = 0,
        CORRIDOR_NAVIGATION = 1,
        TURNING = 2,
        AFTER_TURN_CRAWLING = 3,
        END = 4
    };

    corridor_state corridorState = CALIBRATION;

    double prev_error_ = 0.0;
    double integral_ = 0.0;

    std::atomic<uint8_t> speedLeftWheel{127};
    std::atomic<uint8_t> speedRightWheel{127};

    double startYaw = 0.0;
    int turnDirection_ = -1;
    bool first = true;
    rclcpp::Time last_time;
    rclcpp::Time start_time;
    int counter = 0;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<SharedState> state_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;

    // new variables for imu testing
    double baseYaw_ = 0.0;
    double targetYaw_ = 0.0;
    int headingIndex_ = 0;
    bool baseYawInitialized_ = false;

    int intersectionCounter_ = 0;
    int intersectionIgnoreCounter_ = 0;
    bool intersectionFlag_ = false;

    bool leftWallSeen_ = false;
    bool rightWallSeen_ = false;
};
