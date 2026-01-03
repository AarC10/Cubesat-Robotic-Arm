#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <string>
#include <thread>

class NmeaListenerNode final : public rclcpp::Node {
public:
  explicit NmeaListenerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~NmeaListenerNode() override;

private:
  void readerLoop();
  int openSerial(const std::string &device, int baud);

private:
  std::string port;
  int baud;
  std::string topic;
  int maxLineLen;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

  int fd{-1};
  std::atomic<bool> running{false};
  std::thread readerThread;
};
