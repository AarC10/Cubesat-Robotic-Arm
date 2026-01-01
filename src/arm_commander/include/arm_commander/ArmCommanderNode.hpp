#pragma once

#include <rclcpp/rclcpp.hpp>
#include "arm_msgs/msg/arm_command.hpp"

class ArmCommanderNode : public rclcpp::Node {
public:
    ArmCommanderNode();

    ~ArmCommanderNode() override;

    void handleArmCommand(const arm_msgs::msg::ArmCommand::SharedPtr msg);
    
private:
    uint16_t currentCommandNumber = 0;

    // Targets
    int16_t targetShoulderYaw = 0;
    int16_t targetShoulderPitch = 0;
    int16_t targetElbowAngle = 0;
    int16_t targetWristAngle = 0;

    // Current Command
    int16_t commandShoulderYaw = 0;
    int16_t commandShoulderPitch = 0;
    int16_t commandElbowAngle = 0;
    int16_t commandWristAngle = 0;


    // Tunables
    const int16_t angleIncrement = 5; // degrees
};