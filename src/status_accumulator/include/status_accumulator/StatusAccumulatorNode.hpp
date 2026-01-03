#pragma once

#include <arm_msgs/msg/detail/heartbeat_status__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "arm_msgs/msg/gps_status.hpp"
#include "arm_msgs/msg/heartbeat_status.hpp"


class StatusAccumulatorNode : public rclcpp::Node {
public:
    StatusAccumulatorNode();

    ~StatusAccumulatorNode() override;
    
private:
  // Timer callback
  rclcpp::TimerBase::SharedPtr timer;

  // Pub Topics
  rclcpp::Publisher<arm_msgs::msg::HeartbeatStatus>::SharedPtr
      heartbeatStatusPublisher;
  arm_msgs::msg::HeartbeatStatus heartbeatStatus;
  void publishHeartbeatStatusCallback();

  // Receive Topics
  rclcpp::Subscription<arm_msgs::msg::GpsStatus>::SharedPtr
      gpsStatusSubscription;
  void receiveGpsStatus(const arm_msgs::msg::GpsStatus::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
      batteryStateSubscription;
  void receiveBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // SD Card
  std::string sdCardMountPoint;
  void updateSdCardFillPercent();
};