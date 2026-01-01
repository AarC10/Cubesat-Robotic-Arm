#include "arm_commander/ArmCommanderNode.hpp"

ArmCommanderNode::ArmCommanderNode() : Node("arm_commander") {
    RCLCPP_INFO(this->get_logger(), "ArmCommanderNode has been started.");
}

ArmCommanderNode::~ArmCommanderNode() {
    RCLCPP_INFO(this->get_logger(), "ArmCommanderNode is shutting down.");
}

void ArmCommanderNode::handleArmCommand(const arm_msgs::msg::ArmCommand::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received Arm Command: %d", msg->command_number);
    currentCommandNumber = msg->command_number;
    targetShoulderYaw = msg->shoulder_yaw;
    targetShoulderPitch = msg->shoulder_pitch;
    targetElbowAngle = msg->elbow_angle;
    targetWristAngle = msg->wrist_angle;
}

void ArmCommanderNode::timeoutCallback() {
    incrementToTarget(commandShoulderYaw, targetShoulderYaw);
    incrementToTarget(commandShoulderPitch, targetShoulderPitch);
    incrementToTarget(commandElbowAngle, targetElbowAngle);
    incrementToTarget(commandWristAngle, targetWristAngle);

    if (atTarget() && targetTakePicture) {
        RCLCPP_INFO(this->get_logger(), "At target positions. Taking picture.");
        targetTakePicture = false; // Reset flag
        commandTakePicture = true; // Set command
    } else {
        commandTakePicture = false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Current Commands - Shoulder Yaw: %d, Shoulder Pitch: %d, Elbow Angle: %d, Wrist Angle: %d",
                commandShoulderYaw, commandShoulderPitch, commandElbowAngle, commandWristAngle);
}

void ArmCommanderNode::incrementToTarget(int16_t &current, int16_t target) {
    if (current < target) {
        current += angleIncrement;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= angleIncrement;
        if (current < target) {
            current = target;
        }
    }
}

bool ArmCommanderNode::atTarget() const {
    return (commandShoulderYaw == targetShoulderYaw) &&
           (commandShoulderPitch == targetShoulderPitch) &&
           (commandElbowAngle == targetElbowAngle) &&
           (commandWristAngle == targetWristAngle);
}