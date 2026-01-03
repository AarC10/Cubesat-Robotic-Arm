#pragma once

#include <rclcpp/rclcpp.hpp>
#include "arm_msgs/msg/arm_command.hpp"

class StatusAccumulatorNode : public rclcpp::Node {
public:
    StatusAccumulatorNode();

    ~StatusAccumulatorNode() override;

    void receiveArmTarget(const arm_msgs::msg::ArmCommand::SharedPtr msg);
    
private:
    uint16_t currentCommandNumber = 0;

    // Targets
    int16_t targetShoulderYaw = 0;
    int16_t targetShoulderPitch = 0;
    int16_t targetElbowAngle = 0;
    int16_t targetWristAngle = 0;
    bool targetTakePicture = false;

    // Current Command
    int16_t commandShoulderYaw = 0;
    int16_t commandShoulderPitch = 0;
    int16_t commandElbowAngle = 0;
    int16_t commandWristAngle = 0;
    bool commandTakePicture = false;

    // Tunables
    const int16_t angleIncrement; // degrees
    const int timeoutIntervalMs; // milliseconds

    // Timer callback
    rclcpp::TimerBase::SharedPtr timer;

    // Topics
    rclcpp::Subscription<arm_msgs::msg::ArmCommand>::SharedPtr targetSubscription;
    rclcpp::Publisher<arm_msgs::msg::ArmCommand>::SharedPtr commandPublisher;

    void timeoutCallback();

    void incrementToTarget(int16_t &current, int16_t target);

    bool atTarget() const;
};