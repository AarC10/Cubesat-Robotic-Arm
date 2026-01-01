#include "arm_commander/ArmCommanderNode.hpp"

ArmCommanderNode::ArmCommanderNode(const std::string receiveTopic, const std::string publishTopic, const int16_t angleIncrement, const int timeoutIntervalMs) : Node("arm_commander"), angleIncrement(angleIncrement), timeoutIntervalMs(timeoutIntervalMs) {
    timer = this->create_wall_timer(
        std::chrono::milliseconds(timeoutIntervalMs),
        std::bind(&ArmCommanderNode::timeoutCallback, this)
    );

    targetSubscription = this->create_subscription<arm_msgs::msg::ArmCommand>(
        receiveTopic,
        10,
        std::bind(&ArmCommanderNode::handleArmCommand, this, std::placeholders::_1)
    );
    commandPublisher = this->create_publisher<arm_msgs::msg::ArmCommand>(publishTopic, 10);
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

    auto commandMsg = arm_msgs::msg::ArmCommand();
    commandMsg.command_number = currentCommandNumber;
    commandMsg.shoulder_yaw = commandShoulderYaw;
    commandMsg.shoulder_pitch = commandShoulderPitch;
    commandMsg.elbow_angle = commandElbowAngle;
    commandMsg.wrist_angle = commandWristAngle;
    commandMsg.take_picture = commandTakePicture;
    commandPublisher->publish(commandMsg);
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