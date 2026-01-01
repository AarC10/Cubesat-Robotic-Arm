#include "arm_commander/ArmCommanderNode.hpp"

ArmCommanderNode::ArmCommanderNode() : Node("arm_commander") {
    RCLCPP_INFO(this->get_logger(), "ArmCommanderNode has been started.");
}

ArmCommanderNode::~ArmCommanderNode() {
    RCLCPP_INFO(this->get_logger(), "ArmCommanderNode is shutting down.");
}

void ArmCommanderNode::handleArmCommand(const arm_msgs::msg::ArmCommand::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received Arm Command: %d", msg->command_number);
}

