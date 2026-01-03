#include "status_accumulator/StatusAccumulatorNode.hpp"

#include <string>
#include <sys/statvfs.h>

StatusAccumulatorNode::StatusAccumulatorNode() : Node("status_accumulator") {
  const auto heartbeatPublishIntervalMs =
      this->declare_parameter<int>("heartbeat_publish_interval_ms", 1000);
  const auto gpsStatusTopic =
      this->declare_parameter<std::string>("gps_status_topic", "/gps/status");
  const auto batteryStateTopic = this->declare_parameter<std::string>(
      "battery_state_topic", "/sensor_msgs/msg/BatteryState");

  timer = this->create_wall_timer(
      std::chrono::milliseconds(heartbeatPublishIntervalMs),
      std::bind(&StatusAccumulatorNode::publishHeartbeatStatusCallback, this));

  gpsStatusSubscription = this->create_subscription<arm_msgs::msg::GpsStatus>(
      gpsStatusTopic, 10,
      std::bind(&StatusAccumulatorNode::receiveGpsStatus, this,
                std::placeholders::_1));
  batteryStateSubscription =
      this->create_subscription<sensor_msgs::msg::BatteryState>(
          batteryStateTopic, 10,
          std::bind(&StatusAccumulatorNode::receiveBatteryState, this,
                    std::placeholders::_1));

  heartbeatStatusPublisher =
      this->create_publisher<arm_msgs::msg::HeartbeatStatus>("heartbeat_status",
                                                             10);

  sdCardMountPoint = this->declare_parameter<std::string>("sd_card_mount_point",
                                                          "/mnt/sdcard");

  RCLCPP_INFO(this->get_logger(), "StatusAccumulatorNode has been started.");
}

StatusAccumulatorNode::~StatusAccumulatorNode() {
  RCLCPP_INFO(this->get_logger(), "StatusAccumulatorNode is shutting down.");
}

void StatusAccumulatorNode::publishHeartbeatStatusCallback() {
  heartbeatStatus.uptime = static_cast<uint32_t>(this->now().seconds());
  updateSdCardFillPercent();
  heartbeatStatusPublisher->publish(heartbeatStatus);
}

void StatusAccumulatorNode::receiveGpsStatus(
    const arm_msgs::msg::GpsStatus::SharedPtr msg) {
  heartbeatStatus.gps_time = msg->timestamp;
  heartbeatStatus.latitude = msg->latitude;
  heartbeatStatus.longitude = msg->longitude;
}

void StatusAccumulatorNode::receiveBatteryState(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  heartbeatStatus.battery_voltage = msg->voltage;
  heartbeatStatus.current_draw = msg->current;
}

void StatusAccumulatorNode::updateSdCardFillPercent() {
  struct statvfs stat;
  if (statvfs(sdCardMountPoint.c_str(), &stat) != 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to get filesystem statistics for %s",
                 sdCardMountPoint.c_str());
    return;
  }

  uint64_t totalBlocks = stat.f_blocks;
  uint64_t freeBlocks = stat.f_bfree;
  uint64_t usedBlocks = totalBlocks - freeBlocks;

  if (totalBlocks == 0) {
    RCLCPP_WARN(this->get_logger(), "Total blocks is zero for filesystem %s",
                sdCardMountPoint.c_str());
    heartbeatStatus.sd_card_fill_percentage = 0;
    return;
  }

  uint8_t fillPercentage =
      static_cast<uint8_t>((usedBlocks * 100) / totalBlocks);
  heartbeatStatus.sd_card_fill_percentage = fillPercentage;
}