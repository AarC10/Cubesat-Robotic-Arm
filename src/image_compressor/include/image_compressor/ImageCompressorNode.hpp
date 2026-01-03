#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageCompressorNode final : public rclcpp::Node {
public:
  explicit ImageCompressorNode();
  ~ImageCompressorNode() override;

private:
  // Pub Subs
  rclcpp::Subscription<bool>::SharedPtr imageRequestSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rawImageSub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressedImagePub;

  sensor_msgs::msg::Image::SharedPtr lastRawImage;

  // Callbacks
  void handleImageRequest(const bool msg);
  void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
};
