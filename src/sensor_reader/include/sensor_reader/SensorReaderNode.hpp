#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_reader/adxl375.hpp"
#include "sensor_reader/ina260.hpp"

class SensorReaderNode : public rclcpp::Node {
public:
  SensorReaderNode(
      const std::string &ina260I2cDev = "/dev/i2c-1",
      const std::string &ina260Topic = "/sensor_msgs/msg/Ina260",
      uint8_t ina260Addr = 0x40,
      const std::string &adxl375I2cDev = "/dev/i2c-1",
      const std::string &adxl375Topic = "/sensor_msgs/msg/BatteryState",
      uint8_t adxl375Addr = 0x53, int readIntervalMs = 1000);

  ~SensorReaderNode() override;

private:
  void timeoutCallback();

  rclcpp::TimerBase::SharedPtr timer;
  int readIntervalMs;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr adxl375Publisher;
  Adxl375 adxl375;
  bool adxl375Initialized{false};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr ina260Publisher;
  Ina260 ina260;
  bool ina260Initialized{false};
};